package kr.arcadia.arcPathFinding.policy.node

import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import kr.arcadia.arcPathFinding.query.BlockQuery

data class NodeAttrs(
    val isLadder: BooleanArray, // 노드 위치가 사다리
    val danger: IntArray        // 위험 가중치(주변 용암/가시 등) - 정수 페널티
)

fun buildNodeAttrs(g: NavWorldGraph, bq: BlockQuery): NodeAttrs {
    val n = g.nodeCount
    val ladder = BooleanArray(n)
    val danger = IntArray(n)

    for (u in 0 until n) {
        val x = g.xyzt[u*3]; val y = g.xyzt[u*3+1]; val z = g.xyzt[u*3+2]
        // 사다리: 바로 그 칸만 1회 체크
        ladder[u] = bq.ladder(x, y, z)
        // 위험: 주변 3x3 바닥(y-1)만 간단 집계
        var pen = 0
        for (ox in -1..1) for (oz in -1..1) {
            if (bq.liquid(x+ox, y-1, z+oz)) pen += 20  // 용암/물 가까움 페널티(서버 정책에 맞게)
        }
        danger[u] = pen
    }
    return NodeAttrs(ladder, danger)
}