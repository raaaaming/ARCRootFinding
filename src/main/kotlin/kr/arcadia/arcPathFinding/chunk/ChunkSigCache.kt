package kr.arcadia.arcPathFinding.chunk

import java.io.DataInputStream
import java.io.DataOutputStream
import java.nio.file.Files
import java.nio.file.Path
import java.util.zip.GZIPInputStream
import java.util.zip.GZIPOutputStream

class ChunkSigCache {
    private val map = HashMap<Long, ChunkSig>() // key = (cx<<32)^(cz&0xffffffff)
    private fun key(cx:Int, cz:Int) = (cx.toLong() shl 32) xor (cz.toLong() and 0xFFFFFFFFL)

    fun get(cx:Int, cz:Int): ChunkSig? = map[key(cx,cz)]
    fun put(sig: ChunkSig) { map[key(sig.cx, sig.cz)] = sig }

    fun save(path: Path) {
        Files.newOutputStream(path).use { raw ->
            GZIPOutputStream(raw).use { gz ->
                DataOutputStream(gz).use { out ->
                    out.writeInt(0x574E4D53) // "WNMS"
                    out.writeInt(map.size)
                    for ((_, s) in map) {
                        out.writeInt(s.cx); out.writeInt(s.cz)
                        out.writeInt(s.yMin); out.writeInt(s.yMax)
                        out.writeInt(s.policyHash)
                        out.writeInt(s.standableCrc)
                    }
                }
            }
        }
    }
    fun load(path: Path) {
        if (!Files.exists(path)) return
        Files.newInputStream(path).use { raw ->
            GZIPInputStream(raw).use { gz ->
                DataInputStream(gz).use { `in` ->
                    val magic = `in`.readInt()
                    require(magic == 0x574E4D53) { "Bad magic for WNM sig cache" }
                    val n = `in`.readInt()
                    repeat(n) {
                        val cx = `in`.readInt(); val cz = `in`.readInt()
                        val yMin = `in`.readInt(); val yMax = `in`.readInt()
                        val ph = `in`.readInt(); val crc = `in`.readInt()
                        put(ChunkSig(cx,cz,yMin,yMax,ph,crc))
                    }
                }
            }
        }
    }
}