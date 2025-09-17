package kr.arcadia.arcPathFinding.core

import java.util.PriorityQueue
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

//private fun octileHeuristic3D(ax:Int,ay:Int,az:Int, bx:Int,by:Int,bz:Int,
//                              base:Int = 10, diagExtra:Int = 4, vert:Int = 1): Int {
//    val dx = abs(bx-ax); val dz = abs(bz-az)
//    val dy = abs(by-ay)
//    val dmin = min(dx, dz); val dmax = max(dx, dz)
//    val hXZ = dmin * (base + diagExtra) + (dmax - dmin) * base
//    val hY = dy * vert // 낙하가 더 싸더라도 admissible 보장 위해 최소 단가 사용
//    return hXZ + hY
//}
//
///** 양방향 A*: f = g + h, 정지조건은 best <= min(fF + fB) 근사 */
//fun QueryEngine.routeAStar(sx:Int, sy:Int, sz:Int, tx:Int, ty:Int, tz:Int,
//                           base:Int = 10, diagExtra:Int = 4, vert:Int = 1): List<IntArray> {
//    val s = nearestNode(sx,sy,sz)
//    val t = nearestNode(tx,ty,tz)
//    if (s < 0 || t < 0) return emptyList()
//
//    val n = g.nodeCount
//    val INF = Int.MAX_VALUE/4
//    data class QN(val f:Int, val g:Int, val u:Int)
//
//    val distF = IntArray(n) { INF }; val distB = IntArray(n) { INF }
//    val prevF = IntArray(n) { -1 }; val prevB = IntArray(n) { -1 }
//    val pqF = PriorityQueue<QN>(compareBy { it.f })
//    val pqB = PriorityQueue<QN>(compareBy { it.f })
//
//    distF[s] = 0
//    pqF.add(QN(octileHeuristic3D(g.nodeX(s),g.nodeY(s),g.nodeZ(s), tx,ty,tz, base,diagExtra,vert), 0, s))
//    distB[t] = 0
//    pqB.add(QN(octileHeuristic3D(g.nodeX(t),g.nodeY(t),g.nodeZ(t), sx,sy,sz, base,diagExtra,vert), 0, t))
//
//    var best = INF; var meet = -1
//
//    while (pqF.isNotEmpty() || pqB.isNotEmpty()) {
//        if (pqF.isNotEmpty()) {
//            val cur = pqF.poll()
//            if (cur.g != distF[cur.u]) { /*skip*/ } else {
//                val u = cur.u
//                if (distF[u] + distB[u] < best) { best = distF[u] + distB[u]; meet = u }
//                var i = idx.upOff[u]; val e = idx.upOff[u+1]
//                while (i < e) {
//                    val v = idx.upTo[i]
//                    val wuv = idx.upW[i]
//                    if (wuv < INF && distF[v] > distF[u] + wuv) {
//                        distF[v] = distF[u] + wuv
//                        prevF[v] = u
//                        val h = octileHeuristic3D(g.nodeX(v),g.nodeY(v),g.nodeZ(v), tx,ty,tz, base,diagExtra,vert)
//                        pqF.add(QN(distF[v] + h, distF[v], v))
//                    }
//                    i++
//                }
//            }
//        }
//        if (pqB.isNotEmpty()) {
//            val cur = pqB.poll()
//            if (cur.g != distB[cur.u]) { /*skip*/ } else {
//                val u = cur.u
//                if (distF[u] + distB[u] < best) { best = distF[u] + distB[u]; meet = u }
//                var i = revOff[u]; val e = revOff[u+1]
//                while (i < e) {
//                    val v = revTo[i]
//                    val eid = edgeIndex(v, u)
//                    val wvu = if (eid>=0) idx.upW[eid] else INF
//                    if (wvu < INF && distB[v] > distB[u] + wvu) {
//                        distB[v] = distB[u] + wvu
//                        prevB[v] = u
//                        val h = octileHeuristic3D(g.nodeX(v),g.nodeY(v),g.nodeZ(v), sx,sy,sz, base,diagExtra,vert)
//                        pqB.add(QN(distB[v] + h, distB[v], v))
//                    }
//                    i++
//                }
//            }
//        }
//        // 근사 정지: 두 PQ의 최소 f가 best 이상이면 멈춤
//        val fF = pqF.peek()?.f ?: INF
//        val fB = pqB.peek()?.f ?: INF
//        if (min(fF, fB) >= best) break
//    }
//
//    if (meet == -1) return emptyList()
//
//    // (이하는 기존 route()와 동일) — Upward 경로 복원 & 언패킹
//    val upPath = ArrayList<Int>()
//    run {
//        val stack = ArrayList<Int>()
//        var u = meet
//        while (u != -1) { stack.add(u); u = prevF[u] }
//        for (i in stack.size-1 downTo 0) upPath.add(stack[i])
//    }
//    run {
//        var u = prevB[meet]
//        while (u != -1) { upPath.add(u); u = prevB[u] }
//    }
//
//    val seq = ArrayList<Int>()
//    if (upPath.isNotEmpty()) {
//        seq.add(upPath[0])
//        for (i in 0 until upPath.size-1) {
//            val a = upPath[i]; val b = upPath[i+1]
//            val lo = if (idx.rank[a] < idx.rank[b]) a else b
//            val hi = if (idx.rank[a] < idx.rank[b]) b else a
//            unpackAscending(lo, hi, seq)
//        }
//    }
//    return seq.map { intArrayOf(g.nodeX(it), g.nodeY(it), g.nodeZ(it)) }
//}