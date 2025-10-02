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
        if (map.isEmpty()) return -1

        val cx0 = floorDiv(x, chunkSize); val cz0 = floorDiv(z, chunkSize)
        var best = -1; var bestD = Long.MAX_VALUE

        fun consider(nodeId:Int) {
            val dx = (g.nodeX(nodeId) - x).toLong()
            val dy = (g.nodeY(nodeId) - y).toLong()
            val dz = (g.nodeZ(nodeId) - z).toLong()
            val d2 = dx * dx + dy * dy + dz * dz
            if (d2 < bestD) {
                bestD = d2
                best = nodeId
            }
        }

        fun scanChunk(cx:Int, cz:Int) {
            val arr = map[ChunkKey(cx, cz)] ?: return
            for (u in arr) {
                consider(u)
            }
        }

        // 나선형 확장
        var r = 0
        while (r <= maxRadius) {
            // 위/아래 변
            for (cx in cx0 - r..cx0 + r) {
                scanChunk(cx, cz0 - r)
                if (r > 0) scanChunk(cx, cz0 + r)
            }
            // 좌/우 변
            if (r > 0) {
                for (cz in cz0 - r + 1..<cz0 + r) {
                    scanChunk(cx0 - r, cz)
                    scanChunk(cx0 + r, cz)
                }
            }
            if (best != -1) return best
            r++
        }

        // 반경 내에서 찾지 못한 경우 → 전체 그래프 브루트 포스 fallback
        for (u in 0 until g.nodeCount) {
            consider(u)
        }
        return best
    }

    /** 디버그 용도: 현재 인덱스에 저장된 모든 노드를 출력한다. */
    fun dumpAllNodes(emit: (String) -> Unit = ::println): List<String> {
        val lines = ArrayList<String>()
        var total = 0

        if (map.isEmpty()) {
            lines += "ChunkSpatialIndex: no nodes stored"
        } else {
            val chunks = map.entries
                .sortedWith(compareBy({ it.key.cx }, { it.key.cz }))

            for ((key, nodes) in chunks) {
                lines += "chunk[${key.cx},${key.cz}] (${nodes.size} nodes)"
                val sorted = nodes.sortedArray()
                for (nodeId in sorted) {
                    val x = g.nodeX(nodeId)
                    val y = g.nodeY(nodeId)
                    val z = g.nodeZ(nodeId)
                    lines += "  id=$nodeId -> ($x,$y,$z)"
                }
                total += nodes.size
            }
        }

        lines += "total nodes: $total across ${map.size} chunks"
        lines.forEach(emit)
        return lines
    }
}
