package kr.arcadia.arcPathFinding.core

import kr.arcadia.arcPathFinding.cch.CCHIndex
import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import java.util.PriorityQueue

class QueryEngine(
    private val g: NavWorldGraph,
    private val idx: CCHIndex
) {
    // 전치 그래프(Reverse Upward) 준비
    private val revOff: IntArray
    private val revTo: IntArray
    init {
        val n = g.nodeCount
        val off = idx.upOff; val to = idx.upTo
        val deg = IntArray(n)
        for (u in 0 until n) {
            var i = off[u]; val e = off[u+1]
            while (i < e) { deg[to[i]]++; i++ }
        }
        revOff = IntArray(n+1)
        for (i in 0 until n) revOff[i+1] = revOff[i] + deg[i]
        revTo = IntArray(revOff[n])
        val cur = IntArray(n) { revOff[it] }
        for (u in 0 until n) {
            var i = off[u]; val e = off[u+1]
            while (i < e) {
                val v = to[i]
                revTo[cur[v]++] = u
                i++
            }
        }
    }

    // 최근접 노드 스냅 (단순 버전: 전체 스캔)
    private fun nearestNode(x:Int,y:Int,z:Int): Int {
        var best = -1; var bestD = Long.MAX_VALUE
        val N = g.nodeCount; val a = g.xyzt
        var i=0
        while (i < N) {
            val dx = (a[i*3]-x).toLong(); val dy = (a[i*3+1]-y).toLong(); val dz = (a[i*3+2]-z).toLong()
            val d2 = dx*dx + dy*dy + dz*dz
            if (d2 < bestD) { bestD = d2; best = i }
            i++
        }
        return best
    }

    private fun edgeIndex(u:Int, v:Int): Int {
        val off = idx.upOff; val to = idx.upTo
        var lo = off[u]; var hi = off[u+1]-1
        while (lo <= hi) {
            val m = (lo+hi) ushr 1
            val t = to[m]
            if (t == v) return m
            if (t < v) lo = m+1 else hi = m-1
        }
        return -1
    }

    // Upward 경로 언패킹: [u, ..., v]로 확장
    private fun unpackAscending(u:Int, v:Int, out: MutableList<Int>) {
        // 전제: rank[u] < rank[v]
        val eid = edgeIndex(u, v)
        require(eid >= 0) { "No upward edge for unpack" }
        val mid = idx.upMid[eid]
        if (mid < 0) {
            out.add(v)
            return
        }
        // u->mid 와 mid->v 로 분해 (방향 주의: mid<rank(u))
        // (mid,u)는 상승이 아니므로 (mid,u)로 호출하고 결과를 뒤집어 붙임
        val left = ArrayList<Int>()
        unpackAscending(minOf(mid,u), maxOf(mid,u), left) // [min..max]
        // left는 [mid..u] 또는 [u..mid]
        if (left.first() == u) left.reverse() // [u..mid] 로 맞춤
        val right = ArrayList<Int>()
        unpackAscending(minOf(mid,v), maxOf(mid,v), right) // [mid..v]
        // 병합: u..mid + (mid..v) (mid 중복 제거)
        out.addAll(left.drop(1)) // u는 이미 바깥에서 포함될 것이므로 mid부터
        out.addAll(right)        // right는 mid부터 시작, drop(1) 안 해도 위 줄에서 제거됨
    }

    /** 경로 질의: (sx,sy,sz) -> (tx,ty,tz), 결과는 노드 좌표 리스트 */
    fun route(sx:Int, sy:Int, sz:Int, tx:Int, ty:Int, tz:Int): List<IntArray> {
        val s = nearestNode(sx,sy,sz)
        val t = nearestNode(tx,ty,tz)
        if (s < 0 || t < 0) return emptyList()

        val n = g.nodeCount
        val INF = Int.MAX_VALUE/4
        val distF = IntArray(n) { INF }
        val distB = IntArray(n) { INF }
        val prevF = IntArray(n) { -1 }
        val prevB = IntArray(n) { -1 }

        data class QN(val d:Int, val u:Int)
        val pqF = PriorityQueue<QN>(compareBy { it.d })
        val pqB = PriorityQueue<QN>(compareBy { it.d })

        distF[s] = 0; pqF.add(QN(0,s))
        distB[t] = 0; pqB.add(QN(0,t))

        var best = INF
        var meet = -1

        // 양방향 다익스트라 (Upward / Reverse-Upward)
        while (pqF.isNotEmpty() || pqB.isNotEmpty()) {
            if (pqF.isNotEmpty()) {
                val cur = pqF.poll(); if (cur.d != distF[cur.u]) { /*skip*/ } else {
                    val u = cur.u
                    if (distF[u] + distB[u] < best) { best = distF[u] + distB[u]; meet = u }
                    val sOff = idx.upOff[u]; val sEnd = idx.upOff[u+1]
                    var i = sOff
                    while (i < sEnd) {
                        val v = idx.upTo[i]
                        val wuv = idx.upW[i]
                        if (wuv < INF && distF[v] > distF[u] + wuv) {
                            distF[v] = distF[u] + wuv
                            prevF[v] = u
                            pqF.add(QN(distF[v], v))
                        }
                        i++
                    }
                }
            }
            if (pqB.isNotEmpty()) {
                val cur = pqB.poll(); if (cur.d != distB[cur.u]) { /*skip*/ } else {
                    val u = cur.u
                    if (distF[u] + distB[u] < best) { best = distF[u] + distB[u]; meet = u }
                    val sOff = revOff[u]; val sEnd = revOff[u+1]
                    var i = sOff
                    while (i < sEnd) {
                        val v = revTo[i] // 역방향에서 v->u 이면 원본은 u<-v (즉 forward에서 v는 u보다 낮은 rank)
                        // 역방향도 upW를 그대로 씀(무향 메트릭 가정)
                        val eid = edgeIndex(v, u)
                        val wvu = if (eid>=0) idx.upW[eid] else INF
                        if (wvu < INF && distB[v] > distB[u] + wvu) {
                            distB[v] = distB[u] + wvu
                            prevB[v] = u
                            pqB.add(QN(distB[v], v))
                        }
                        i++
                    }
                }
            }
            // 간단한 정지조건: 현재 best보다 작은 후보가 더 이상 나오기 힘들면 break (생략 가능)
        }
        if (meet == -1) return emptyList()

        // Upward 경로 복원: s -> meet
        val upPath = ArrayList<Int>()
        run {
            val stack = ArrayList<Int>()
            var u = meet
            while (u != -1) { stack.add(u); u = prevF[u] }
            // stack: [meet, ..., s]
            for (i in stack.size-1 downTo 0) upPath.add(stack[i])
        }
        // meet -> t (역방향 prevB 이용)
        run {
            var u = prevB[meet]
            while (u != -1) { upPath.add(u); u = prevB[u] }
        }

        // 언패킹: Upward 경로의 각 간선을 풀어서 원래 노드열로
        val seq = ArrayList<Int>()
        if (upPath.isNotEmpty()) {
            seq.add(upPath[0])
            for (i in 0 until upPath.size-1) {
                val a = upPath[i]; val b = upPath[i+1]
                // a,b의 랭크 방향 맞추기
                val lo = if (idx.rank[a] < idx.rank[b]) a else b
                val hi = if (idx.rank[a] < idx.rank[b]) b else a
                unpackAscending(lo, hi, seq) // seq 마지막에 hi까지 추가
            }
        }

        // 좌표로 변환
        val out = ArrayList<IntArray>(seq.size)
        for (u in seq) {
            out.add(intArrayOf(g.xyzt[u*3], g.xyzt[u*3+1], g.xyzt[u*3+2]))
        }
        return out
    }
}