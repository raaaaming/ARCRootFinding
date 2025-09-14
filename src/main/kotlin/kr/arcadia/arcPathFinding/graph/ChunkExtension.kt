package kr.arcadia.arcPathFinding.graph

import kr.arcadia.arcPathFinding.chunk.NavChunk
import kr.arcadia.arcPathFinding.util.IntArrayList
import kotlin.math.abs

private fun keyXZ(x:Int, z:Int): Long = (x.toLong() shl 32) xor (z.toLong() and 0xFFFFFFFFL)

fun mergeChunks(chunks: List<NavChunk>): NavWorldGraph {
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

    // 3-2) 경계(게이트) 교차 간선
    //
    // 규칙(스켈레톤 단순판):
    // - 인접 청크의 대응 경계와 (x,z)가 1칸 차이(상대 경계 좌표)이며
    // - |Δy| <= 1 이면 연결 (양방향)
    // (더 큰 낙하는 각 청크 내부 그래프에서 이미 처리된다고 가정)
    //
    // 게이트 빠른 매칭을 위해 이웃 청크마다 (x,z)->localNode 리스트 인덱스를 만든다.
    data class GateIndex(val byXZ: MutableMap<Long, MutableList<Int>>)
    fun buildGateIndex(ch: NavChunk, which: Char): GateIndex {
        val map = HashMap<Long, MutableList<Int>>()
        val nodes = when(which) {
            'N' -> ch.gateN
            'S' -> ch.gateS
            'W' -> ch.gateW
            else -> ch.gateE
        }
        for (uLocal in nodes) {
            val x = ch.coords[uLocal*3]
            val z = ch.coords[uLocal*3+2]
            val key = keyXZ(x, z)
            map.computeIfAbsent(key){ mutableListOf() }.add(uLocal)
        }
        return GateIndex(map)
    }

    // 이웃 청크의 게이트 인덱스 캐시
    val gateCache = HashMap<Pair<Int,Char>, GateIndex>() // (chunkIdx, side) -> index

    fun gateIndexOf(chunkIdx:Int, side:Char): GateIndex =
        gateCache.getOrPut(chunkIdx to side) { buildGateIndex(chunks[chunkIdx], side) }

    // 북/남, 서/동 쌍 매칭
    for (i in chunks.indices) {
        val ch = chunks[i]
        val base = offsets[i]

        // ① 북쪽: 이웃은 (cx, cz-1), 현 z=minZ, 이웃 z=maxZ
        idxOf[C(ch.pos.cx, ch.pos.cz - 1)]?.let { nIdx ->
            val neighbor = chunks[nIdx]
            // 이웃의 남쪽 게이트 인덱스(by x,z)
            val gIdx = gateIndexOf(nIdx, 'S')
            for (uLocal in ch.gateN) {
                val ux = ch.coords[uLocal*3]
                val uy = ch.coords[uLocal*3+1]
                val uz = ch.coords[uLocal*3+2]   // == minZ
                // 이웃 쪽 좌표는 (ux, uz-1)
                val key = keyXZ(ux, uz - 1)
                val candidates = gIdx.byXZ[key] ?: continue
                for (vLocal in candidates) {
                    val vy = neighbor.coords[vLocal*3+1]
                    if (abs(vy - uy) <= 1) {
                        val u = base + uLocal
                        val v = offsets[nIdx] + vLocal
                        nbr[u].add(v)
                        nbr[v].add(u)
                    }
                }
            }
        }

        // ② 남쪽: 이웃은 (cx, cz+1), 현 z=maxZ, 이웃 z=minZ
        idxOf[C(ch.pos.cx, ch.pos.cz + 1)]?.let { sIdx ->
            val neighbor = chunks[sIdx]
            val gIdx = gateIndexOf(sIdx, 'N')
            for (uLocal in ch.gateS) {
                val ux = ch.coords[uLocal*3]
                val uy = ch.coords[uLocal*3+1]
                val uz = ch.coords[uLocal*3+2]   // == maxZ
                val key = keyXZ(ux, uz + 1)
                val candidates = gIdx.byXZ[key] ?: continue
                for (vLocal in candidates) {
                    val vy = neighbor.coords[vLocal*3+1]
                    if (abs(vy - uy) <= 1) {
                        val u = base + uLocal
                        val v = offsets[sIdx] + vLocal
                        nbr[u].add(v)
                        nbr[v].add(u)
                    }
                }
            }
        }

        // ③ 서쪽: 이웃은 (cx-1, cz), 현 x=minX, 이웃 x=maxX
        idxOf[C(ch.pos.cx - 1, ch.pos.cz)]?.let { wIdx ->
            val neighbor = chunks[wIdx]
            val gIdx = gateIndexOf(wIdx, 'E')
            for (uLocal in ch.gateW) {
                val ux = ch.coords[uLocal*3]
                val uy = ch.coords[uLocal*3+1]
                val uz = ch.coords[uLocal*3+2]   // == minX 열
                val key = keyXZ(ux - 1, uz)
                val candidates = gIdx.byXZ[key] ?: continue
                for (vLocal in candidates) {
                    val vy = neighbor.coords[vLocal*3+1]
                    if (abs(vy - uy) <= 1) {
                        val u = base + uLocal
                        val v = offsets[wIdx] + vLocal
                        nbr[u].add(v)
                        nbr[v].add(u)
                    }
                }
            }
        }

        // ④ 동쪽: 이웃은 (cx+1, cz), 현 x=maxX, 이웃 x=minX
        idxOf[C(ch.pos.cx + 1, ch.pos.cz)]?.let { eIdx ->
            val neighbor = chunks[eIdx]
            val gIdx = gateIndexOf(eIdx, 'W')
            for (uLocal in ch.gateE) {
                val ux = ch.coords[uLocal*3]
                val uy = ch.coords[uLocal*3+1]
                val uz = ch.coords[uLocal*3+2]   // == maxX 열
                val key = keyXZ(ux + 1, uz)
                val candidates = gIdx.byXZ[key] ?: continue
                for (vLocal in candidates) {
                    val vy = neighbor.coords[vLocal*3+1]
                    if (abs(vy - uy) <= 1) {
                        val u = base + uLocal
                        val v = offsets[eIdx] + vLocal
                        nbr[u].add(v)
                        nbr[v].add(u)
                    }
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