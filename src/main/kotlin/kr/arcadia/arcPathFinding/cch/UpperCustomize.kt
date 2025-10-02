package kr.arcadia.arcPathFinding.cch

import kr.arcadia.arcPathFinding.graph.NavWorldGraph

fun customizeUpperTrianglePass(idx: CCHIndex) {
    val n = idx.rank.size
    val rank = idx.rank
    val off = idx.upOff; val to = idx.upTo; val w = idx.upW

    // 높은 rank부터 역순
    val nodes = IntArray(n) { it }.sortedByDescending { rank[it] }
    for (v in nodes) {
        val s = off[v]; val e = off[v+1]
        var i = s
        while (i < e) {
            val a = to[i]; val w_va = w[i]
            var j = i + 1
            while (j < e) {
                val b = to[j]; val w_vb = w[j]
                val (lo, hi) = if (rank[a] < rank[b]) a to b else b to a
                val eid = edgeIndex(idx, lo, hi)
                if (eid >= 0) {
                    val cand = if (w_va == Int.MAX_VALUE/4 || w_vb == Int.MAX_VALUE/4) Int.MAX_VALUE/4 else w_va + w_vb
                    if (cand < w[eid]) w[eid] = cand
                }
                j++
            }
            i++
        }
    }
}

fun customizeWeightsPerfectFast(
    idx: CCHIndex,
    g: NavWorldGraph,
    weightFn: (NavWorldGraph, Int, Int)->Int
) {
    val n = g.nodeCount
    val off = idx.upOff; val to = idx.upTo; val mid = idx.upMid; val w = idx.upW
    val rank = idx.rank
    val INF = Int.MAX_VALUE / 4

    // 0) 초기화: 원본 간선은 weightFn, 쇼트컷은 INF
    for (u in 0 until n) {
        var i = off[u]; val e = off[u+1]
        while (i < e) {
            val v = to[i]
            w[i] = if (mid[i] == -1) weightFn(g, u, v) else INF
            i++
        }
    }

    // 랭크 기준 노드 순서
    val nodesAsc = IntArray(n){ it }.sortedBy { rank[it] }
    val nodesDesc = nodesAsc.asReversed()

    // 1) Lower triangles: rank(u) < rank(v) < rank(w)
    for (u in nodesAsc) {
        val sU = off[u]; val eU = off[u+1]
        var iUV = sU
        while (iUV < eU) {
            val v = to[iUV]
            // Up(u)의 [iUV+1 ..) 과 Up(v)의 [..]
            var iUW = iUV + 1
            var iVW = off[v]
            val eVW = off[v+1]
            while (iUW < eU && iVW < eVW) {
                val wU = to[iUW]; val wV = to[iVW]
                when {
                    wU < wV -> iUW++
                    wU > wV -> iVW++
                    else -> {
                        // u->wU 간선 완화: w[u->wU] = min(w[u->wU], w[u->v] + w[v->wU])
                        val cand = safePlus(w[iUV], w[iVW], INF)
                        if (cand < w[iUW]) w[iUW] = cand
                        iUW++; iVW++
                    }
                }
            }
            iUV++
        }
    }

    // 2) Upper triangles: rank(u) < rank(w) < rank(v)
    for (u in nodesDesc) {
        val sU = off[u]; val eU = off[u+1]
        var iUV = sU
        while (iUV < eU) {
            val v = to[iUV]
            // Up(u)의 [iUV+1 ..) 과 Up(v)의 [..] 교집합을 보되,
            // 이번엔 v->w 간선을 완화: w[v->w] = min(w[v->w], w[u->w] + w[u->v])
            var iUW = iUV + 1
            var iVW = off[v]
            val eVW = off[v+1]
            while (iUW < eU && iVW < eVW) {
                val wU = to[iUW]; val wV = to[iVW]
                when {
                    wU < wV -> iUW++
                    wU > wV -> iVW++
                    else -> {
                        val cand = safePlus(w[iUW], w[iUV], INF)
                        if (cand < w[iVW]) w[iVW] = cand
                        iUW++; iVW++
                    }
                }
            }
            iUV++
        }
    }
}

private fun safePlus(a:Int, b:Int, INF:Int) =
    if (a >= INF || b >= INF) INF else a + b