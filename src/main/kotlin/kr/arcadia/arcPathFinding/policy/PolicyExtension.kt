package kr.arcadia.arcPathFinding.policy

import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import kr.arcadia.arcPathFinding.policy.node.NodeAttrs
import kotlin.math.abs
import kotlin.math.max


fun makeAttrWeightFn(attrs: NodeAttrs, policy: WeightPolicy): (NavWorldGraph, Int, Int) -> Int {
    return { g, u, v ->
        val ux = g.xyzt[u*3]; val uy = g.xyzt[u*3+1]; val uz = g.xyzt[u*3+2]
        val vx = g.xyzt[v*3]; val vy = g.xyzt[v*3+1]; val vz = g.xyzt[v*3+2]
        val dx = abs(vx-ux); val dz = abs(vz-uz); val dy = vy-uy

        var w = policy.base
        if (dx==1 && dz==1) w += policy.diagExtra
        if (dy== 1) w += policy.stepUp
        if (dy<  0) w += policy.fallPer * abs(dy)

        // 사다리 보너스(둘 중 하나라도 사다리면)
        if ((dx==0 && dz==0 && abs(dy)==1) && (attrs.isLadder[u] || attrs.isLadder[v])) {
            w = max(1, w + policy.ladderBias)
        }

        // 위험(최소값 채택)
        w += minOf(attrs.danger[u], attrs.danger[v])

        (w * policy.nightMultiplier).toInt()
    }
}