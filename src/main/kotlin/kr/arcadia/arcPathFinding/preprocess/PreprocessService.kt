package kr.arcadia.arcPathFinding.preprocess

import kr.arcadia.arcPathFinding.chunk.ChunkPos
import kr.arcadia.arcPathFinding.chunk.ChunkSig
import kr.arcadia.arcPathFinding.chunk.ChunkSigCache
import kr.arcadia.arcPathFinding.chunk.NavChunk
import kr.arcadia.arcPathFinding.chunk.NavChunkBuilder
import kr.arcadia.arcPathFinding.policy.MovePolicy
import kr.arcadia.arcPathFinding.query.BlockQuery

class PreprocessService(
    private val bq: BlockQuery,
    private val policy: MovePolicy
) {
    /** 중심 (x0,z0)와 반경(청크)으로 전처리. 달라진 청크만 재생성하여 반환 */
    fun preprocessRadius(
        x0:Int, z0:Int, radiusChunks:Int,
        yMin:Int, yMax:Int,
        existing: ChunkSigCache
    ): List<NavChunk> {
        val out = ArrayList<NavChunk>()
        val c0x = Math.floorDiv(x0, 16); val c0z = Math.floorDiv(z0, 16)
        // 나선형 순회
        var r = 0
        loop@ while (r <= radiusChunks) {
            var x = -r
            while (x <= r) {
                val ring = listOf(
                    c0x + x to c0z - r,
                    c0x + x to c0z + r
                )
                for ((cx, cz) in ring) {
                    if (Math.abs(cx - c0x) > r || Math.abs(cz - c0z) > r) continue
                    if (r == 0 && (cx != c0x || cz != c0z)) continue
                    // 시그니처 비교
                    val mask = NavChunkBuilder.computeStandableMask(bq, ChunkPos(cx,cz), yMin, yMax)
                    val crc = mask.crc32()
                    val old = existing.get(cx, cz)
                    val cur = ChunkSig(cx, cz, yMin, yMax, policy.hash(), crc)
                    if (old == null || old != cur) {
                        val ch = NavChunkBuilder.buildFromMask(bq, ChunkPos(cx,cz), mask, policy)
                        out.add(ch)
                        existing.put(cur)
                    }
                }
                x++
            }
            // 좌/우 변
            var z = -r+1
            while (z <= r-1) {
                val ring = listOf(
                    c0x - r to c0z + z,
                    c0x + r to c0z + z
                )
                for ((cx, cz) in ring) {
                    val mask = NavChunkBuilder.computeStandableMask(bq, ChunkPos(cx,cz), yMin, yMax)
                    val crc = mask.crc32()
                    val old = existing.get(cx, cz)
                    val cur = ChunkSig(cx, cz, yMin, yMax, policy.hash(), crc)
                    if (old == null || old != cur) {
                        val ch = NavChunkBuilder.buildFromMask(bq, ChunkPos(cx,cz), mask, policy)
                        out.add(ch)
                        existing.put(cur)
                    }
                }
                z++
            }
            r++
        }
        return out
    }
}