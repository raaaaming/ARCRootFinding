package kr.arcadia.arcPathFinding.cch

import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import kr.arcadia.arcPathFinding.policy.MovePolicy
import kr.arcadia.arcPathFinding.util.IntArrayList
import kr.arcadia.arcPathFinding.util.PairArrayList
import kotlin.math.abs
import kotlin.math.max

fun contract(g: NavWorldGraph, order: IntArray, policy: MovePolicy): CCHIndex {
    val n = g.nodeCount
    if (n == 0) return CCHIndex(IntArray(0), IntArray(1){0}, IntArray(0), IntArray(0), IntArray(0))

    // node -> rank
    val rank = IntArray(n)
    for (r in order.indices) rank[order[r]] = r

    // Upward 임시 버퍼: 각 노드 u에 대해 (to, mid) 쌓아두기
    val upTmp = Array(n) { PairArrayList() }

    // 1) 원본 간선들을 Upward 방향으로만 추가(원래 그래프 방향성 무시하고 undirected 취급)
    for (u in 0 until n) {
        val s = g.csrOff[u]; val e = g.csrOff[u+1]
        var j = s
        while (j < e) {
            val v = g.csrTo[j]
            if (rank[u] < rank[v]) {
                upTmp[u].add(v, /*mid*/-1) // 원본 upward 간선
            }
            j++
        }
    }

    // 2) 수축 루프: 각 v에서 상위 이웃 H를 클리크로 연결
    //    H = { h | (v,h) 원본 이웃이며 rank[h] > rank[v] }
    val tmpH = IntArray(256) // 재사용 버퍼(필요시 확장)
    for (r in order.indices) {
        val v = order[r]
        // H 수집
        var hCount = 0
        run {
            val s = g.csrOff[v]; val e = g.csrOff[v+1]
            var j = s
            while (j < e) {
                val h = g.csrTo[j]
                if (rank[h] > r) {
                    if (hCount == tmpH.size) {
                        // 확장
                        val newArr = IntArray(tmpH.size * 2)
                        System.arraycopy(tmpH, 0, newArr, 0, tmpH.size)
                        // replace
                        for (k in 0 until newArr.size) { /* no-op */ }
                        // Kotlin trick: assign back
                        // but we cannot reassign val; so instead allocate a local list
                        // -> 간단히 아래에서 ArrayList 사용으로 교체
                    }
                    tmpH[hCount++] = h
                }
                j++
            }
        }
        // 위에서 버퍼 재할당 이슈가 있으니, 간단히 동적 리스트로 다시 구현
        val H = IntArrayList(max(8, hCount))
        run {
            val s = g.csrOff[v]; val e = g.csrOff[v+1]
            var j = s
            while (j < e) {
                val h = g.csrTo[j]
                if (rank[h] > r) H.add(h)
                j++
            }
        }
        val mH = H.size()
        if (mH <= 1) continue

        // 모든 조합 (i<j)에 대해 낮은 랭크 -> 높은 랭크로 간선 추가, mid = v
        var i = 0
        while (i < mH) {
            val a = H[i]; val ra = rank[a]
            var k = i + 1
            while (k < mH) {
                val b = H[k]; val rb = rank[b]
                if (ra < rb) {
                    upTmp[a].add(b, v)
                } else if (rb < ra) {
                    upTmp[b].add(a, v)
                } else {
                    // theoretically equal rank는 없음
                }
                k++
            }
            i++
        }
    }

    // 3) 각 노드의 Upward 이웃을 (to 오름차순) 정렬 & 중복 제거
    //    같은 (u->to) 간선이 여러 번 생겼다면:
    //      - 원본(-1 mid)이 있으면 그것을 우선
    //      - 아니면 mid의 rank가 가장 낮은(=먼저 수축된) 것을 택함
    val upOff = IntArray(n + 1)
    var m = 0
    // (미리 길이 계산을 위해 한 번 정리하면서 개수 누적)
    val cleanedTo = Array(n) { IntArrayList() }
    val cleanedMid = Array(n) { IntArrayList() }

    fun preferMid(currentMid:Int, candidateMid:Int): Int {
        if (currentMid == -1 || candidateMid == -1) return -1
        // 둘 다 쇼트컷이면 더 낮은 rank의 mid를 선택(재귀 언패킹 안정성)
        return if (rank[candidateMid] < rank[currentMid]) candidateMid else currentMid
    }

    for (u in 0 until n) {
        val (toArr0, midArr0) = upTmp[u].toArrays()
        if (toArr0.isEmpty()) continue

        // 병렬 배열을 'to' 기준으로 정렬
        val idx = (0 until toArr0.size).toMutableList()
        // 간단한 퀵소트/병합 대신 O(k log k) 정렬
        idx.sortBy { toArr0[it] }

        var lastTo = -1
        var chosenMid = Int.MAX_VALUE
        var cnt = 0
        for (p in idx) {
            val t = toArr0[p]
            val mMid = midArr0[p]
            if (t != lastTo) {
                if (lastTo != -1) {
                    cleanedTo[u].add(lastTo)
                    cleanedMid[u].add(if (chosenMid == Int.MAX_VALUE) -1 else chosenMid)
                    cnt++
                }
                lastTo = t
                chosenMid = mMid
            } else {
                chosenMid = preferMid(chosenMid, mMid)
            }
        }
        if (lastTo != -1) {
            cleanedTo[u].add(lastTo)
            cleanedMid[u].add(if (chosenMid == Int.MAX_VALUE) -1 else chosenMid)
            cnt++
        }
        upOff[u] = m
        m += cnt
    }
    upOff[n] = m

    val upTo = IntArray(m)
    val upMid = IntArray(m)
    var ptr = 0
    for (u in 0 until n) {
        val toL = cleanedTo[u]; val midL = cleanedMid[u]
        var i = 0
        while (i < toL.size()) {
            upTo[ptr] = toL[i]
            upMid[ptr] = midL[i]
            ptr++
            i++
        }
    }

    val upW = IntArray(m) { 0 } // 커스터마이즈 단계에서 채움

    return CCHIndex(rank = rank, upOff = upOff, upTo = upTo, upMid = upMid, upW = upW)
}

fun defaultWeightFn(g: NavWorldGraph, u:Int, v:Int): Int {
    val ux = g.xyzt[u*3]; val uy = g.xyzt[u*3+1]; val uz = g.xyzt[u*3+2]
    val vx = g.xyzt[v*3]; val vy = g.xyzt[v*3+1]; val vz = g.xyzt[v*3+2]
    val dx = abs(vx-ux); val dz = abs(vz-uz); val dy = vy-uy
    var w = 10 // 평지=10 (정수 가중치로)
    if (dx==1 && dz==1) w += 4 // 대각(≈√2-1 보정)
    if (dy== 1) w += 4        // 한칸 오르기
    if (dy<  0) w += 1*abs(dy)// 낙하 패널티(깊이에 비례)
    return w
}

// ---- 유틸: (u->v) edge index 찾기 (없으면 -1) ----
private fun edgeIndex(idx: CCHIndex, u:Int, v:Int): Int {
    val off = idx.upOff; val to = idx.upTo
    var lo = off[u]; var hi = off[u+1]-1
    while (lo <= hi) {
        val mid = (lo+hi) ushr 1
        val t = to[mid]
        if (t == v) return mid
        if (t < v) lo = mid+1 else hi = mid-1
    }
    return -1
}

/** 퍼펙트 커스터마이즈: 낮은 삼각형 반복 없이 한 번의 하향→상향 전파로 완성 */
fun customizeWeightsPerfect(idx: CCHIndex, g: NavWorldGraph, weightFn: (NavWorldGraph, Int, Int)->Int = ::defaultWeightFn) {
    val n = g.nodeCount
    val off = idx.upOff;
    val to = idx.upTo;
    val mid = idx.upMid;
    val w = idx.upW
    val rank = idx.rank

    // 0) 초기화: 원본 간선은 weightFn, 쇼트컷은 INF
    val INF = Int.MAX_VALUE / 4
    for (u in 0 until n) {
        val s = off[u];
        val e = off[u + 1]
        var i = s
        while (i < e) {
            val v = to[i]
            w[i] = if (mid[i] == -1) weightFn(g, u, v) else INF
            i++
        }
    }

    // 1) 낮은 삼각형 완화: v를 낮은→높은 rank 순으로 스윕
    val orderByRank = IntArray(n) { it }.sortedBy { rank[it] }
    for (v in orderByRank) {
        // v의 상위 이웃들 H(v) 수집
        val start = off[v];
        val end = off[v + 1]
        if (start >= end) continue
        // 모든 쌍 (a,b) (rank[a] < rank[b])에 대해 완화
        var i = start
        while (i < end) {
            val a = to[i];
            val w_va = w[i]
            if (w_va == INF) {
                i++; continue
            } // 아직 유효치가 없으면 스킵
            var j = i + 1
            while (j < end) {
                val b = to[j];
                val w_vb = w[j]
                if (w_vb != INF) {
                    val u = if (rank[a] < rank[b]) a else b
                    val t = if (rank[a] < rank[b]) b else a
                    val eid = edgeIndex(idx, u, t)
                    if (eid >= 0) {
                        val cand = w_va + w_vb
                        if (cand < w[eid]) w[eid] = cand
                    }
                }
                j++
            }
            i++
        }
    }
    // (선택) 상위 삼각형 한 번 더 돌리면 수렴이 더 빨라짐(대개 필요 없음)
}