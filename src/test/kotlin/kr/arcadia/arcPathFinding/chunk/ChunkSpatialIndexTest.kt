package kr.arcadia.arcPathFinding.chunk

import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import kotlin.test.Test
import kotlin.test.assertEquals

class ChunkSpatialIndexTest {
    @Test
    fun `nearestNode falls back to full scan when local radius is empty`() {
        val coords = intArrayOf(
            0, 64, 0,
            128, 64, 0
        )
        val chunkOf = intArrayOf(0, 1)
        val csrOff = intArrayOf(0, 0, 0)
        val csrTo = IntArray(0)
        val graph = NavWorldGraph(2, coords, chunkOf, csrOff, csrTo)
        val index = ChunkSpatialIndex(graph)

        val result = index.nearestNode(400, 64, 0)

        assertEquals(1, result, "Expected the fallback scan to select the closest node")
    }

    @Test
    fun `nearestNode returns minus one when graph is empty`() {
        val graph = NavWorldGraph(0, IntArray(0), IntArray(0), intArrayOf(0), IntArray(0))
        val index = ChunkSpatialIndex(graph)

        val result = index.nearestNode(0, 64, 0)

        assertEquals(-1, result, "Empty graphs should not produce a node id")
    }
}
