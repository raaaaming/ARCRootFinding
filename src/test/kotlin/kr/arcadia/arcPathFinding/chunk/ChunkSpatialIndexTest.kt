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

    @Test
    fun `dumpAllNodes reports every stored node`() {
        val coords = intArrayOf(
            0, 64, 0,
            16, 65, 0,
            32, 70, 16
        )
        val chunkOf = intArrayOf(0, 1, 2)
        val csrOff = intArrayOf(0, 0, 0, 0)
        val csrTo = IntArray(0)
        val graph = NavWorldGraph(3, coords, chunkOf, csrOff, csrTo)
        val index = ChunkSpatialIndex(graph)

        val lines = index.dumpAllNodes { }

        assertEquals(
            listOf(
                "chunk[0,0] (1 nodes)",
                "  id=0 -> (0,64,0)",
                "chunk[1,0] (1 nodes)",
                "  id=1 -> (16,65,0)",
                "chunk[2,1] (1 nodes)",
                "  id=2 -> (32,70,16)",
                "total nodes: 3 across 3 chunks"
            ),
            lines
        )
    }
}
