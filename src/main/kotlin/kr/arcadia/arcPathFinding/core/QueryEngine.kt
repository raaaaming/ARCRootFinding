package kr.arcadia.arcPathFinding.core

import kr.arcadia.arcPathFinding.cch.CCHIndex
import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import java.util.PriorityQueue

open class QueryEngine(
    open val g: NavWorldGraph,
    open val idx: CCHIndex
) {
    // --------- (옵션) 브루트 최근접 스냅 ---------
    fun nearestNode(x:Int,y:Int,z:Int): Int {
        var best = -1; var bestD = Long.MAX_VALUE
        val a = g.xyzt
        for (u in 0 until g.nodeCount) {
            val dx = (a[u*3]-x).toLong()
            val dy = (a[u*3+1]-y).toLong()
            val dz = (a[u*3+2]-z).toLong()
            val d2 = dx*dx + dy*dy + dz*dz
            if (d2 < bestD) { bestD = d2; best = u }
        }
        return best
    }

    // --------- Upward 간선 검색 (이진 탐색) ---------
    fun edgeIndex(u:Int, v:Int): Int {
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

    // --------- 언패킹 (포함 범위 [lo..hi] 반환) ---------
    /** CCH 표준: rank[lo] < rank[hi] 전제. 결과는 노드열 [lo, ..., hi] */
    private fun unpackInclusive(lo:Int, hi:Int): IntArray {
        val eid = edgeIndex(lo, hi)
        require(eid >= 0) { "Upward edge ($lo->$hi) not found for unpack" }
        val mid = idx.upMid[eid]
        if (mid < 0) {
            return intArrayOf(lo, hi)
        }
        // mid의 rank는 lo보다 더 낮다(= mid < lo < hi). 그러므로 (mid, lo)와 (mid, hi) 둘 다 upward.
        val left  = unpackInclusive(minOf(mid, lo), maxOf(mid, lo))   // [mid..lo]
        val right = unpackInclusive(minOf(mid, hi), maxOf(mid, hi))   // [mid..hi]
        // 병합: [mid..lo] + [mid..hi]에서 mid 중복 제거하고 [lo..mid..hi]가 되도록 순서 맞추기
        // left: [mid, ..., lo]  -> lo가 마지막
        // right:[mid, ..., hi]  -> mid로 시작
        val out = IntArray(left.size + right.size - 1)
        // left를 그대로 복사
        System.arraycopy(left, 0, out, 0, left.size)
        // right는 mid 중복을 제거하고 이어붙임
        System.arraycopy(right, 1, out, left.size, right.size - 1)
        return out
    }

    // --------- 공통: s,t (노드ID)로 경로 구해서 좌표 반환 ---------
    private fun routeByNodeIds(s:Int, t:Int): List<IntArray> {
        if (s == t) return listOf(intArrayOf(g.xyzt[s*3], g.xyzt[s*3+1], g.xyzt[s*3+2]))

        val n = g.nodeCount
        val INF = Int.MAX_VALUE/4
        data class QN(val d:Int, val u:Int)

        val distF = IntArray(n) { INF }
        val distB = IntArray(n) { INF }
        val prevF = IntArray(n) { -1 }
        val prevB = IntArray(n) { -1 }

        val pqF = PriorityQueue<QN>(compareBy { it.d })
        val pqB = PriorityQueue<QN>(compareBy { it.d })

        distF[s] = 0; pqF.add(QN(0, s))
        distB[t] = 0; pqB.add(QN(0, t))

        var best = INF
        var meet = -1

        // 양방향 다익스트라 (Forward=Upward / Backward=Upward from target)
        while (pqF.isNotEmpty() || pqB.isNotEmpty()) {
            if (pqF.isNotEmpty()) {
                val cur = pqF.poll()
                if (cur.d == distF[cur.u]) {
                    val u = cur.u
                    if (distF[u] + distB[u] < best) { best = distF[u] + distB[u]; meet = u }
                    var i = idx.upOff[u]; val e = idx.upOff[u+1]
                    while (i < e) {
                        val v = idx.upTo[i]
                        val w = idx.upW[i]
                        if (w < INF && distF[v] > distF[u] + w) {
                            distF[v] = distF[u] + w
                            prevF[v] = u
                            pqF.add(QN(distF[v], v))
                        }
                        i++
                    }
                }
            }
            if (pqB.isNotEmpty()) {
                val cur = pqB.poll()
                if (cur.d == distB[cur.u]) {
                    val u = cur.u
                    if (distF[u] + distB[u] < best) { best = distF[u] + distB[u]; meet = u }
                    var i = idx.upOff[u]; val e = idx.upOff[u+1]
                    while (i < e) {
                        val v = idx.upTo[i]
                        val w = idx.upW[i]
                        if (w < INF && distB[v] > distB[u] + w) {
                            distB[v] = distB[u] + w
                            prevB[v] = u
                            pqB.add(QN(distB[v], v))
                        }
                        i++
                    }
                }
            }
            // (선택) 더 똑똑한 정지조건 가능: 두 큐의 top 합이 best 이상이면 break
        }

        if (meet == -1) return emptyList()

        // Upward 경로 노드열(피크를 중심으로 앞뒤 결합)
        val upPath = ArrayList<Int>()
        run {
            // s -> meet (forward)
            val stack = ArrayList<Int>()
            var u = meet
            while (u != -1) { stack.add(u); u = prevF[u] }
            for (i in stack.size-1 downTo 0) upPath.add(stack[i]) // [s..meet]
        }
        run {
            // meet -> t (backward)
            var u = prevB[meet]
            while (u != -1) { upPath.add(u); u = prevB[u] } // [meet..t]
        }

        // 언패킹: 각 (a,b) 구간을 rank 방향에 맞게 펼친다
        val seq = ArrayList<Int>()
        if (upPath.isNotEmpty()) {
            seq.add(upPath[0])
            for (i in 0 until upPath.size-1) {
                val a = upPath[i]
                val b = upPath[i+1]
                val ra = idx.rank[a]; val rb = idx.rank[b]
                val lo = if (ra < rb) a else b
                val hi = if (ra < rb) b else a
                val expanded = unpackInclusive(lo, hi) // [lo..hi]
                if (ra < rb) {
                    // a -> b가 상승 구간: [lo..hi]에서 앞의 한 칸(lo)만 중복 제거하고 붙임
                    for (k in 1 until expanded.size) seq.add(expanded[k])
                } else {
                    // a -> b가 하강 구간: [lo..hi]를 뒤집어서 [hi..lo]로, hi==a이므로 앞 한 칸 제거
                    for (k in expanded.size-2 downTo 0) seq.add(expanded[k])
                }
            }
        }

        // 좌표로 변환
        val out = ArrayList<IntArray>(seq.size)
        for (u in seq) {
            out.add(intArrayOf(g.xyzt[u*3], g.xyzt[u*3+1], g.xyzt[u*3+2]))
        }
        return out
    }

    // --------- 기존 API: 좌표로 스냅(브루트) + 탐색 ---------
    fun route(sx:Int, sy:Int, sz:Int, tx:Int, ty:Int, tz:Int): List<IntArray> {
        val s = nearestNode(sx,sy,sz)
        val t = nearestNode(tx,ty,tz)
        if (s < 0 || t < 0) return emptyList()
        return routeByNodeIds(s, t)
    }

    // --------- 요청한 API: 외부 스냅 함수 사용 ---------
    /**
     * @param snapFn (x,y,z) -> global node id (없으면 음수)
     * @return 경로의 (x,y,z) 좌표 리스트. 없으면 빈 리스트.
     */
    fun routeWithSnap(
        snapFn:(Int,Int,Int)->Int,
        sx:Int, sy:Int, sz:Int,
        tx:Int, ty:Int, tz:Int
    ): List<IntArray> {
        val s = snapFn(sx,sy,sz)
        val t = snapFn(tx,ty,tz)
        if (s < 0 || t < 0) return emptyList()
        return routeByNodeIds(s, t)
    }
}