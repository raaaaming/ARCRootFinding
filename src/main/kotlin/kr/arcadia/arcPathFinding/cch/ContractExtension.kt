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
        // H = Upward 이웃(= rank > r)
        val startV = g.csrOff[v]; val endV = g.csrOff[v+1]
        // v의 상위 이웃 목록 만들기(정렬 필요 시 정렬)
        val H = IntArrayList()
        var j = startV; while (j < endV) { val h = g.csrTo[j]; if (rank[h] > r) H.add(h); j++ }

        // a ∈ H에 대해: Up(a) ∩ H 교집합 걷기 → (a,b)만 쇼트컷 추가
        for (ix in 0 until H.size()) {
            val a = H[ix]
            // Up(a)는 g.csrOff[a].. 가 아니라, 이미 upTmp(원본 upward)에도 들어가 있음.
            // 여기서는 원본 그래프의 인접 리스트로 충분.
            var pA = g.csrOff[a]; val eA = g.csrOff[a+1]
            // H를 "집합"으로 빠르게 조회하려면 boolean mark 사용
            // (스코프 내에서만 쓰는 작은 비트셋)
            // -> 간단히 해시셋도 가능
            while (pA < eA) {
                val b = g.csrTo[pA]
                if (rank[b] > r && /* b ∈ H */ true) {
                    // (a,b) 추가: a<b 랭크 방향으로만
                    if (rank[a] < rank[b]) upTmp[a].add(b, v) else upTmp[b].add(a, v)
                }
                pA++
            }
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
fun edgeIndex(idx: CCHIndex, u:Int, v:Int): Int {
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

fun contractFast(g: NavWorldGraph, order:IntArray, policy: MovePolicy): CCHIndex {
    val n = g.nodeCount
    if (n==0) return CCHIndex(IntArray(0), IntArray(1){0}, IntArray(0), IntArray(0), IntArray(0))

    val rank = IntArray(n); for (r in order.indices) rank[order[r]] = r
    val upTmp = Array(n){ PairArrayList() }

    // 1) 원본 간선 → Upward only
    for (u in 0 until n) {
        var i = g.csrOff[u]; val e = g.csrOff[u+1]
        while (i < e) {
            val v = g.csrTo[i]
            if (rank[u] < rank[v]) upTmp[u].add(v, -1)
            i++
        }
    }

    // 2) 수축: 세트 마킹 방식
    val inH = BooleanArray(n)
    for (r in order.indices) {
        val v = order[r]
        // H(v) 수집 & 마킹
        val H = IntArrayList()
        run {
            var i = g.csrOff[v]; val e = g.csrOff[v+1]
            while (i < e) {
                val h = g.csrTo[i]
                if (rank[h] > r) { H.add(h); inH[h] = true }
                i++
            }
        }
        // 각 a∈H의 이웃 b 중에서 b∈H이고 a≠b인 것만 쇼트컷
        var ix = 0
        while (ix < H.size()) {
            val a = H[ix]
            var i = g.csrOff[a]; val e = g.csrOff[a+1]
            while (i < e) {
                val b = g.csrTo[i]
                if (b != a && inH[b]) {
                    if (rank[a] < rank[b]) upTmp[a].add(b, v) else upTmp[b].add(a, v)
                }
                i++
            }
            ix++
        }
        // 언마킹
        ix = 0; while (ix < H.size()) { inH[H[ix]] = false; ix++ }
    }

    // 3) 정렬+중복제거(원본 간선 우선)
    val upOff = IntArray(n+1); var M = 0
    val cleanedTo = Array(n){ IntArrayList() }
    val cleanedMid = Array(n){ IntArrayList() }
    fun prefer(cur:Int, cand:Int, rank:IntArray): Int {
        if (cur==-1 || cand==-1) return -1
        return if (rank[cand] < rank[cur]) cand else cur
    }
    for (u in 0 until n) {
        val (to0, mid0) = upTmp[u].toArrays()
        if (to0.isEmpty()) { upOff[u] = M; continue }
        // 인덱스 정렬 by dest
        val idx = (to0.indices).toMutableList()
        idx.sortBy { to0[it] }
        var last = -1; var chosen = Int.MAX_VALUE; var cnt = 0
        for (p in idx) {
            val t = to0[p]; val m = mid0[p]
            if (t != last) {
                if (last != -1) { cleanedTo[u].add(last); cleanedMid[u].add(if (chosen==Int.MAX_VALUE) -1 else chosen); cnt++ }
                last = t; chosen = m
            } else {
                chosen = prefer(chosen, m, rank)
            }
        }
        if (last != -1) { cleanedTo[u].add(last); cleanedMid[u].add(if (chosen==Int.MAX_VALUE) -1 else chosen); cnt++ }
        upOff[u] = M; M += cnt
    }
    upOff[n] = M
    val upTo = IntArray(M); val upMid = IntArray(M); var p = 0
    for (u in 0 until n) {
        val toL = cleanedTo[u]; val midL = cleanedMid[u]
        var i=0; while (i<toL.size()) { upTo[p] = toL[i]; upMid[p] = midL[i]; p++; i++ }
    }
    val upW = IntArray(M) { 0 }
    return CCHIndex(rank, upOff, upTo, upMid, upW)
}

private class PairBuf(cap:Int = 8){
    var to = IntArray(cap); var mid = IntArray(cap); var n = 0
    fun add(t:Int,m:Int){
        if (n==to.size){ val nc = max(8,to.size*2); to=to.copyOf(nc); mid=mid.copyOf(nc) }
        to[n]=t; mid[n]=m; n++
    }
}

private fun sortPairsByFirst(to:IntArray, mid:IntArray, len:Int){
    fun swap(i:Int,j:Int){ val a=to[i]; to[i]=to[j]; to[j]=a; val b=mid[i]; mid[i]=mid[j]; mid[j]=b }
    fun q(l:Int,r:Int){
        var i=l; var j=r; val p=to[(l+r) ushr 1]
        while(i<=j){
            while(to[i] < p) i++
            while(to[j] > p) j--
            if(i<=j){ swap(i,j); i++; j-- }
        }
        if(l<j) q(l,j); if(i<r) q(i,r)
    }
    if (len>1) q(0,len-1)
}

fun contractUltra(g: NavWorldGraph, order:IntArray, policy: MovePolicy): CCHIndex {
    val n = g.nodeCount
    if (n==0) return CCHIndex(IntArray(0), IntArray(1){0}, IntArray(0), IntArray(0), IntArray(0))

    // node -> rank
    val rank = IntArray(n); for (r in order.indices) rank[order[r]] = r

    // ① Up 인접리스트 구성 (원본 CSR 한 번 스캔)
    val upCnt = IntArray(n+1)
    for (u in 0 until n){
        var i=g.csrOff[u]; val e=g.csrOff[u+1]
        while(i<e){ val v=g.csrTo[i]; if (rank[u] < rank[v]) upCnt[u+1]++; i++ }
    }
    // prefix-sum
    for (i in 0 until n) upCnt[i+1]+=upCnt[i]
    val upTo = IntArray(upCnt[n])
    run {
        val cur = upCnt.clone()
        for (u in 0 until n){
            var i=g.csrOff[u]; val e=g.csrOff[u+1]
            while(i<e){ val v=g.csrTo[i]; if (rank[u] < rank[v]) upTo[cur[u]++] = v; i++ }
        }
    }

    // ② 원본 Up 간선은 그대로 유지해야 하므로 버퍼에 먼저 적재
    val buf = Array(n){ PairBuf() }
    for (u in 0 until n){
        var i=upCnt[u]; val e=upCnt[u+1]
        while(i<e){ buf[u].add(upTo[i], -1); i++ } // -1 = 원본 간선
    }

    // ③ 수축 루프: H(v)=Up(v) 마킹, a∈H의 Up(a)∩H(v)만 걷기
    val inH = BooleanArray(n)
    for (r in order.indices){
        val v = order[r]
        // H(v) 수집/마킹
        var s = upCnt[v]; val t = upCnt[v+1]
        var i = s; while (i < t){ inH[upTo[i]] = true; i++ }
        // a ∈ H(v)
        i = s
        while (i < t){
            val a = upTo[i]
            // Up(a)와 H(v) 교차
            var p = upCnt[a]; val q = upCnt[a+1]
            while (p < q){
                val b = upTo[p]
                if (inH[b]) {
                    // a<b 방향으로만 추가
                    if (rank[a] < rank[b]) buf[a].add(b, v) else buf[b].add(a, v)
                }
                p++
            }
            i++
        }
        // 언마킹
        i = s; while (i < t){ inH[upTo[i]] = false; i++ }
    }

    // ④ 노드별 정렬+중복제거(원본 우선, 아니면 mid rank 낮은 쪽)
    val off = IntArray(n+1); var M = 0
    val outTo = IntArrayList(); val outMid = IntArrayList()
    fun prefer(cur:Int, cand:Int): Int {
        if (cur==-1 || cand==-1) return -1
        return if (rank[cand] < rank[cur]) cand else cur
    }
    for (u in 0 until n){
        val b = buf[u]
        // to,mid를 dest 기준 정렬
        sortPairsByFirst(b.to, b.mid, b.n)
        var last = Int.MIN_VALUE
        var chosen = Int.MAX_VALUE
        var k = 0
        while (k < b.n){
            val dest = b.to[k]; val m = b.mid[k]
            if (dest != last){
                if (last != Int.MIN_VALUE){
                    outTo.add(last); outMid.add(if (chosen==Int.MAX_VALUE) -1 else chosen)
                }
                last = dest; chosen = m
            } else {
                chosen = prefer(chosen, m)
            }
            k++
        }
        if (last != Int.MIN_VALUE){
            outTo.add(last); outMid.add(if (chosen==Int.MAX_VALUE) -1 else chosen)
        }
        off[u] = M
        M = outTo.size()
    }
    off[n] = M

    val upToFinal  = outTo.toIntArray()
    val upMidFinal = outMid.toIntArray()
    val upW = IntArray(M) { 0 }

    return CCHIndex(rank, off, upToFinal, upMidFinal, upW)
}