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
        val policyHash = policy.hash()

        fun rebuildIfChanged(cx:Int, cz:Int) {
            val pos = ChunkPos(cx, cz)
            val mask = NavChunkBuilder.computeStandableMask(bq, pos, yMin, yMax)
            val crc = mask.crc32()
            val cur = ChunkSig(cx, cz, yMin, yMax, policyHash, crc)
            val old = existing.get(cx, cz)
            if (old == null || old != cur) {
                val ch = NavChunkBuilder.buildFromMask(bq, pos, mask, policy)
                out.add(ch)
                existing.put(cur)
            }
        }

        var r = 0
        while (r <= radiusChunks) {
            var x = -r
            while (x <= r) {
                val cx = c0x + x
                val northZ = c0z - r
                val southZ = c0z + r
                if (Math.abs(cx - c0x) <= r) {
                    if (r == 0) {
                        rebuildIfChanged(cx, northZ)
                    } else {
                        rebuildIfChanged(cx, northZ)
                        rebuildIfChanged(cx, southZ)
                    }
                }
                x++
            }
            if (r > 0) {
                var z = -r + 1
                while (z <= r - 1) {
                    val cz = c0z + z
                    rebuildIfChanged(c0x - r, cz)
                    rebuildIfChanged(c0x + r, cz)
                    z++
                }
            }
            r++
        }
        return out
    }
}