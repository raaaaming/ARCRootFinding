package kr.arcadia.arcPathFinding.graph

import kr.arcadia.arcPathFinding.chunk.NavChunk
import kr.arcadia.arcPathFinding.policy.MovePolicy
import kr.arcadia.arcPathFinding.util.IntArrayList

private fun keyXZ(x:Int, z:Int): Long = (x.toLong() shl 32) xor (z.toLong() and 0xFFFFFFFFL)

fun mergeChunks(chunks: List<NavChunk>, policy: MovePolicy): NavWorldGraph {

    fun pickBestY(uy:Int, ys:IntArray, stepUp:Int, maxDrop:Int): Int? {
        if (ys.isEmpty()) return null
        val seen = java.util.HashSet<Int>(ys.size)
        for (v in ys) seen.add(v)
        if (seen.contains(uy)) return uy
        for (d in 1..stepUp) if (seen.contains(uy + d)) return uy + d
        for (d in 1..maxDrop) if (seen.contains(uy - d)) return uy - d
        return null
    }

    if (chunks.isEmpty()) return NavWorldGraph(0, IntArray(0), IntArray(0), IntArray(1){0}, IntArray(0))

    // 0) 청크 인덱스 맵 (cx,cz) -> idx
    data class C(val cx:Int, val cz:Int)
    val idxOf = HashMap<C, Int>(chunks.size)
    chunks.forEachIndexed { i, ch -> idxOf[C(ch.pos.cx, ch.pos.cz)] = i }

    // 1) 글로벌 노드 오프셋
    val offsets = IntArray(chunks.size + 1)
    for (i in chunks.indices) offsets[i+1] = offsets[i] + chunks[i].nodeCount
    val N = offsets.last()

    // 2) 전역 좌표/소속 채우기
    val xyzt = IntArray(N * 3)
    val chunkOf = IntArray(N)
    for (i in chunks.indices) {
        val ch = chunks[i]
        val base = offsets[i]
        // coords는 [x,y,z]×localN
        System.arraycopy(ch.coords, 0, xyzt, base*3, ch.coords.size)
        java.util.Arrays.fill(chunkOf, base, base + ch.nodeCount, i)
    }

    // 3) 임시 인접 리스트(전역) — 로컬 + 교차간선 모두 쌓고 CSR로 압축
    val nbr = Array(N) { IntArrayList() }

    // 3-1) 로컬 간선 복사 (로컬 -> 글로벌 id 변환)
    for (i in chunks.indices) {
        val ch = chunks[i]; val base = offsets[i]
        val off = ch.adjOff; val to = ch.adjTo
        for (uLocal in 0 until ch.nodeCount) {
            val u = base + uLocal
            val s = off[uLocal]; val e = off[uLocal+1]
            var j = s
            while (j < e) {
                val vLocal = to[j]
                val v = base + vLocal
                nbr[u].add(v)
                j++
            }
        }
    }

    // 3-2) 경계(게이트) 교차 간선 — ★ y상승/하강/대각까지 열어줌
    data class GateIndex(val byXZ: MutableMap<Long, MutableList<Int>>)
    fun buildGateIndex(ch: NavChunk, which: Char): GateIndex {
        val map = HashMap<Long, MutableList<Int>>()
        val nodes = when(which) {
            'N' -> ch.gateN; 'S' -> ch.gateS; 'W' -> ch.gateW; else -> ch.gateE
        }
        for (uLocal in nodes) {
            val x = ch.coords[uLocal*3]
            val z = ch.coords[uLocal*3+2]
            map.computeIfAbsent(keyXZ(x, z)) { mutableListOf() }.add(uLocal)
        }
        return GateIndex(map)
    }
    val gateCache = HashMap<Pair<Int,Char>, GateIndex>()
    fun gateIndexOf(chunkIdx:Int, side:Char) =
        gateCache.getOrPut(chunkIdx to side) { buildGateIndex(chunks[chunkIdx], side) }

// 방향 후보(대각 허용 시 ±1 포함)
    val dxCand = if (policy.allowDiag) intArrayOf(-1,0,1) else intArrayOf(0)
    val dzCand = dxCand

    for (i in chunks.indices) {
        val A = chunks[i]
        val baseA = offsets[i]

        // 북쪽: neighbor (cx, cz-1), A의 z=minZ → B의 z=maxZ
        idxOf[C(A.pos.cx, A.pos.cz - 1)]?.let { idxB ->
            val B = chunks[idxB]; val baseB = offsets[idxB]
            val gB = gateIndexOf(idxB, 'S')
            for (uLocal in A.gateN) {
                val ux = A.coords[uLocal*3]; val uy = A.coords[uLocal*3+1]
                val uz = A.coords[uLocal*3+2] // == minZ
                val tz = uz - 1
                for (dx in dxCand) {
                    val key = keyXZ(ux + dx, tz)
                    val cand = gB.byXZ[key] ?: continue
                    val yList = IntArray(cand.size) { k -> B.coords[cand[k]*3 + 1] }
                    val ty = pickBestY(uy, yList, policy.stepUp, policy.maxDrop) ?: continue
                    val vLocal = cand.first { B.coords[it*3 + 1] == ty }
                    val u = baseA + uLocal; val v = baseB + vLocal
                    nbr[u].add(v); nbr[v].add(u)
                }
            }
        }

        // 남쪽: neighbor (cx, cz+1), A의 z=maxZ → B의 z=minZ
        idxOf[C(A.pos.cx, A.pos.cz + 1)]?.let { idxB ->
            val B = chunks[idxB]; val baseB = offsets[idxB]
            val gB = gateIndexOf(idxB, 'N')
            for (uLocal in A.gateS) {
                val ux = A.coords[uLocal*3]; val uy = A.coords[uLocal*3+1]
                val uz = A.coords[uLocal*3+2] // == maxZ
                val tz = uz + 1
                for (dx in dxCand) {
                    val key = keyXZ(ux + dx, tz)
                    val cand = gB.byXZ[key] ?: continue
                    val yList = IntArray(cand.size) { k -> B.coords[cand[k]*3 + 1] }
                    val ty = pickBestY(uy, yList, policy.stepUp, policy.maxDrop) ?: continue
                    val vLocal = cand.first { B.coords[it*3 + 1] == ty }
                    val u = baseA + uLocal; val v = baseB + vLocal
                    nbr[u].add(v); nbr[v].add(u)
                }
            }
        }

        // 서쪽: neighbor (cx-1, cz), A의 x=minX → B의 x=maxX
        idxOf[C(A.pos.cx - 1, A.pos.cz)]?.let { idxB ->
            val B = chunks[idxB]; val baseB = offsets[idxB]
            val gB = gateIndexOf(idxB, 'E')
            for (uLocal in A.gateW) {
                val ux = A.coords[uLocal*3]; val uy = A.coords[uLocal*3+1]
                val uz = A.coords[uLocal*3+2] // == minX 열
                val tx = ux - 1
                for (dz in dzCand) {
                    val key = keyXZ(tx, uz + dz)
                    val cand = gB.byXZ[key] ?: continue
                    val yList = IntArray(cand.size) { k -> B.coords[cand[k]*3 + 1] }
                    val ty = pickBestY(uy, yList, policy.stepUp, policy.maxDrop) ?: continue
                    val vLocal = cand.first { B.coords[it*3 + 1] == ty }
                    val u = baseA + uLocal; val v = baseB + vLocal
                    nbr[u].add(v); nbr[v].add(u)
                }
            }
        }

        // 동쪽: neighbor (cx+1, cz), A의 x=maxX → B의 x=minX
        idxOf[C(A.pos.cx + 1, A.pos.cz)]?.let { idxB ->
            val B = chunks[idxB]; val baseB = offsets[idxB]
            val gB = gateIndexOf(idxB, 'W')
            for (uLocal in A.gateE) {
                val ux = A.coords[uLocal*3]; val uy = A.coords[uLocal*3+1]
                val uz = A.coords[uLocal*3+2] // == maxX 열
                val tx = ux + 1
                for (dz in dzCand) {
                    val key = keyXZ(tx, uz + dz)
                    val cand = gB.byXZ[key] ?: continue
                    val yList = IntArray(cand.size) { k -> B.coords[cand[k]*3 + 1] }
                    val ty = pickBestY(uy, yList, policy.stepUp, policy.maxDrop) ?: continue
                    val vLocal = cand.first { B.coords[it*3 + 1] == ty }
                    val u = baseA + uLocal; val v = baseB + vLocal
                    nbr[u].add(v); nbr[v].add(u)
                }
            }
        }
    }

    // 4) (선택) 간단한 중복 제거 — 동일 이웃 중복을 줄여 CSR 크기 감소
    //    비용-효과를 고려해서 가벼운 방식으로 처리
    for (u in 0 until N) {
        val list = nbr[u]
        if (list.size() <= 1) continue
        val arr = list.toIntArray()
        java.util.Arrays.sort(arr)
        var m = 0
        for (i in arr.indices) {
            if (i == 0 || arr[i] != arr[i-1]) arr[m++] = arr[i]
        }
        // 재적재
        val nl = IntArrayList(m)
        var i = 0; while (i < m) { nl.add(arr[i]); i++ }
        nbr[u] = nl
    }

    // 5) CSR 압축
    val off = IntArray(N + 1); var M = 0
    for (u in 0 until N) { off[u] = M; M += nbr[u].size() }
    off[N] = M
    val to = IntArray(M)
    for (u in 0 until N) {
        val base = off[u]; val list = nbr[u]
        var j = 0; while (j < list.size()) { to[base + j] = list[j]; j++ }
    }

    return NavWorldGraph(
        nodeCount = N,
        xyzt = xyzt,
        chunkOf = chunkOf,
        csrOff = off,
        csrTo = to
    )
}