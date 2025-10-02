package kr.arcadia.arcPathFinding.core

import kr.arcadia.arcPathFinding.cch.CCHIndex
import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals

class QueryEngineTest {

    @Test
    fun `route climbs up-and-down section`() {
        val (g, idx) = simpleGraph()
        val engine = QueryEngine(g, idx)

        val path = engine.route(0, 64, 0, 2, 64, 0)

        assertEquals(
            listOf(
                listOf(0, 64, 0),
                listOf(1, 64, 0),
                listOf(1, 65, 0),
                listOf(2, 64, 0)
            ),
            path.map { it.toList() }
        )
    }

    private fun simpleGraph(): Pair<NavWorldGraph, CCHIndex> {
        val coords = intArrayOf(
            0, 64, 0,
            1, 64, 0,
            1, 65, 0,
            2, 64, 0
        )
        val csrOff = intArrayOf(0, 1, 3, 5, 6)
        val csrTo = intArrayOf(
            1,
            0, 2,
            1, 3,
            2
        )
        val g = NavWorldGraph(
            nodeCount = 4,
            xyzt = coords,
            chunkOf = intArrayOf(0, 0, 0, 0),
            csrOff = csrOff,
            csrTo = csrTo
        )

        val rank = intArrayOf(0, 1, 3, 2)
        val upOff = intArrayOf(0, 1, 2, 2, 3)
        val upTo = intArrayOf(1, 2, 2)
        val upMid = intArrayOf(-1, -1, -1)
        val upW = intArrayOf(1, 1, 1)
        val idx = CCHIndex(rank, upOff, upTo, upMid, upW)

        return g to idx
    }
}
