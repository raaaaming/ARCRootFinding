package kr.arcadia.arcPathFinding.chunk

import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import java.lang.Math.floorDiv

class ChunkSpatialIndex(
    private val g: NavWorldGraph,
    private val chunkSize:Int = 16
) {
    private val map = HashMap<ChunkKey, IntArray>()

    init {
        val buckets = HashMap<ChunkKey, MutableList<Int>>()
        for (u in 0 until g.nodeCount) {
            val cx = floorDiv(g.nodeX(u), chunkSize)
            val cz = floorDiv(g.nodeZ(u), chunkSize)
            buckets.computeIfAbsent(ChunkKey(cx,cz)){ mutableListOf() }.add(u)
        }
        for ((k, lst) in buckets) map[k] = lst.toIntArray()
    }

    /** 중심 청크에서 시작해 반경 r까지 확장하며 최근접 노드 탐색 */
    fun nearestNode(x:Int, y:Int, z:Int, maxRadius:Int = 3): Int {
        val cx0 = floorDiv(x, chunkSize); val cz0 = floorDiv(z, chunkSize)
        var best = -1; var bestD = Long.MAX_VALUE

        fun scanChunk(cx:Int, cz:Int) {
            val arr = map[ChunkKey(cx,cz)] ?: return
            for (u in arr) {
                val dx = (g.nodeX(u)-x).toLong()
                val dy = (g.nodeY(u)-y).toLong()
                val dz = (g.nodeZ(u)-z).toLong()
                val d2 = dx*dx + dy*dy + dz*dz
                if (d2 < bestD) { bestD = d2; best = u }
            }
        }

        // 나선형 확장
        var r = 0
        while (r <= maxRadius) {
            // 위/아래 변
            for (cx in cx0-r..cx0+r) { scanChunk(cx, cz0-r); if (r>0) scanChunk(cx, cz0+r) }
            // 좌/우 변
            for (cz in cz0-r+1..<cz0+r) { if (r>0) { scanChunk(cx0-r, cz); scanChunk(cx0+r, cz) } }
            if (best != -1) break
            r++
        }
        return best
    }
}