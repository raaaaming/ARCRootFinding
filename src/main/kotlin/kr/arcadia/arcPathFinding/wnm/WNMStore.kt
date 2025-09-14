package kr.arcadia.arcPathFinding.wnm

import kr.arcadia.arcPathFinding.chunk.ChunkPos
import kr.arcadia.arcPathFinding.chunk.NavChunk
import java.io.DataInputStream
import java.io.DataOutputStream
import java.nio.file.Files
import java.nio.file.Path
import java.util.UUID
import java.util.zip.GZIPInputStream
import java.util.zip.GZIPOutputStream

private const val WNM_MAGIC = 0x574E4D31

class WnmStore {

    /** 새 파일에 헤더+청크들(순차 섹션) 저장 */
    fun writeNew(path: Path, header: WNMHeader, chunks: List<NavChunk>) {
        Files.newOutputStream(path).use { raw ->
            GZIPOutputStream(raw).use { gz ->
                DataOutputStream(gz).use { out ->
                    out.writeInt(WNM_MAGIC)
                    out.writeInt(header.version)
                    out.writeInt(header.policyHash)
                    out.writeLong(header.worldUUID.mostSignificantBits)
                    out.writeLong(header.worldUUID.leastSignificantBits)
                    out.writeInt(header.yMin); out.writeInt(header.yMax)
                    out.writeLong(header.createdAt)
                    out.writeInt(chunks.size)

                    for (ch in chunks) writeChunk(out, ch)
                }
            }
        }
    }

    /** 파일에서 헤더만 빠르게 읽기 */
    fun loadHeader(path: Path): WNMHeader {
        Files.newInputStream(path).use { raw ->
            GZIPInputStream(raw).use { gz ->
                DataInputStream(gz).use { `in` ->
                    val magic = `in`.readInt(); require(magic == WNM_MAGIC) { "Bad WNM magic" }
                    val ver = `in`.readInt()
                    val ph = `in`.readInt()
                    val msb = `in`.readLong(); val lsb = `in`.readLong()
                    val yMin = `in`.readInt(); val yMax = `in`.readInt()
                    val created = `in`.readLong()
                    val chunkCount = `in`.readInt()
                    // 스킵: 청크는 건너뜀
                    return WNMHeader(ver, ph, UUID(msb, lsb), yMin, yMax, created)
                }
            }
        }
    }

    /** 전체 로드(헤더 + 모든 청크) */
    fun readAll(path: Path): Pair<WNMHeader, List<NavChunk>> {
        Files.newInputStream(path).use { raw ->
            GZIPInputStream(raw).use { gz ->
                DataInputStream(gz).use { `in` ->
                    val magic = `in`.readInt(); require(magic == WNM_MAGIC) { "Bad WNM magic" }
                    val ver = `in`.readInt()
                    val ph = `in`.readInt()
                    val msb = `in`.readLong(); val lsb = `in`.readLong()
                    val yMin = `in`.readInt(); val yMax = `in`.readInt()
                    val created = `in`.readLong()
                    val cnt = `in`.readInt()

                    val header = WNMHeader(ver, ph, UUID(msb, lsb), yMin, yMax, created)
                    val list = ArrayList<NavChunk>(cnt)
                    repeat(cnt) { list.add(readChunk(`in`)) }
                    return header to list
                }
            }
        }
    }

    /** 증분 패치: 기존 파일을 읽어 (cx,cz) 키로 병합 후 temp에 새로 써서 교체 */
    fun appendOrPatch(path: Path, header: WNMHeader, updated: List<NavChunk>) {
        val (oldHeader, oldChunks) =
            if (Files.exists(path)) readAll(path) else header to emptyList()

        // 헤더 검증(정책/월드/y범위 다르면 전체 재생성 권장)
        require(oldChunks.isEmpty() || (oldHeader.policyHash == header.policyHash && oldHeader.worldUUID == header.worldUUID &&
                oldHeader.yMin == header.yMin && oldHeader.yMax == header.yMax)) {
            "Header mismatch: rebuild WNM from scratch"
        }

        val map = HashMap<Long, NavChunk>(oldChunks.size + updated.size)
        fun key(cx:Int, cz:Int) = (cx.toLong() shl 32) xor (cz.toLong() and 0xFFFFFFFFL)
        for (ch in oldChunks) map[key(ch.pos.cx, ch.pos.cz)] = ch
        for (ch in updated) map[key(ch.pos.cx, ch.pos.cz)] = ch // 업데이트가 우선

        val merged = map.values.sortedWith(compareBy({ it.pos.cx }, { it.pos.cz }))

        val tmp = path.resolveSibling(path.fileName.toString() + ".tmp")
        writeNew(tmp, header.copy(createdAt = System.currentTimeMillis()), merged)
        java.nio.file.Files.move(tmp, path, java.nio.file.StandardCopyOption.REPLACE_EXISTING, java.nio.file.StandardCopyOption.ATOMIC_MOVE)
    }

    // --- 내부: 청크 섹션 (순차 기록/복원) ---

    private fun writeChunk(out: DataOutputStream, ch: NavChunk) {
        out.writeInt(ch.pos.cx); out.writeInt(ch.pos.cz)
        out.writeInt(ch.standableCrc)

        // coords
        out.writeInt(ch.coords.size); for (v in ch.coords) out.writeInt(v)
        // adjOff / adjTo
        out.writeInt(ch.adjOff.size); for (v in ch.adjOff) out.writeInt(v)
        out.writeInt(ch.adjTo.size);  for (v in ch.adjTo) out.writeInt(v)
        // adjTag (필요 없으면 0으로 저장하거나 size=0)
        out.writeInt(ch.adjTag.size); for (v in ch.adjTag) out.writeShort(v.toInt())
        // gates
        fun wIntArr(a:IntArray){ out.writeInt(a.size); for (v in a) out.writeInt(v) }
        wIntArr(ch.gateN); wIntArr(ch.gateS); wIntArr(ch.gateW); wIntArr(ch.gateE)
    }

    private fun readChunk(`in`: DataInputStream): NavChunk {
        val cx = `in`.readInt(); val cz = `in`.readInt()
        val crc = `in`.readInt()

        fun rIntArr(): IntArray { val n = `in`.readInt(); val a = IntArray(n); for (i in 0 until n) a[i] = `in`.readInt(); return a }
        fun rShortArr(): ShortArray { val n = `in`.readInt(); val a = ShortArray(n); for (i in 0 until n) a[i] = `in`.readShort(); return a }

        val coords = rIntArr()
        val adjOff = rIntArr()
        val adjTo  = rIntArr()
        val adjTag = rShortArr()
        val gN = rIntArr(); val gS = rIntArr(); val gW = rIntArr(); val gE = rIntArr()

        return NavChunk(ChunkPos(cx,cz), coords, adjOff, adjTo, adjTag, gN, gS, gW, gE, crc)
    }
}