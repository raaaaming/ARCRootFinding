package kr.arcadia.arcPathFinding.separator

import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import java.lang.Math.floorDiv
import kotlin.math.max
import kotlin.math.min

fun buildOrderByChunkSeparator(
    g: NavWorldGraph,
    chunkSize: Int = 16,
    sepThickness: Int = 1      // ★ 새 파라미터(기본 1) — 두껍게 2~3 추천
): OrderResult {
    val N = g.nodeCount
    if (N == 0) return OrderResult(IntArray(0), IntArray(0))
    require(sepThickness >= 1)

    data class CK(val cx:Int, val cz:Int)
    val buckets = HashMap<CK, MutableList<Int>>()
    var minCx = Int.MAX_VALUE; var maxCx = Int.MIN_VALUE
    var minCz = Int.MAX_VALUE; var maxCz = Int.MIN_VALUE
    for (u in 0 until N) {
        val cx = floorDiv(g.nodeX(u), chunkSize)
        val cz = floorDiv(g.nodeZ(u), chunkSize)
        buckets.computeIfAbsent(CK(cx,cz)){ mutableListOf() }.add(u)
        minCx = min(minCx, cx); maxCx = max(maxCx, cx)
        minCz = min(minCz, cz); maxCz = max(maxCz, cz)
    }

    fun hasAny(x0:Int,x1:Int,z0:Int,z1:Int): Boolean {
        for ((k, _) in buckets) if (k.cx in x0..x1 && k.cz in z0..z1) return true
        return false
    }

    val order = ArrayList<Int>(N)
    val level = IntArray(N) { -1 }

    fun addNodes(list: List<Int>, depth:Int) {
        for (u in list) if (level[u] == -1) { level[u] = depth; order.add(u) }
    }

    fun addColRange(x0:Int,x1:Int, z0:Int,z1:Int, depth:Int) {
        for (cx in x0..x1) for (cz in z0..z1)
            buckets[CK(cx,cz)]?.let { addNodes(it, depth) }
    }
    fun addRowRange(z0:Int,z1:Int, x0:Int,x1:Int, depth:Int) {
        for (cz in z0..z1) for (cx in x0..x1)
            buckets[CK(cx,cz)]?.let { addNodes(it, depth) }
    }

    fun rec(x0:Int,x1:Int,z0:Int,z1:Int, depth:Int) {
        if (x0>x1 || z0>z1) return
        if (!hasAny(x0,x1,z0,z1)) return

        val w = x1 - x0 + 1
        val h = z1 - z0 + 1
        if (w <= 1 && h <= 1) { addColRange(x0,x1,z0,z1,depth); return }

        val t = sepThickness
        if (w >= h) {
            val mid = (x0 + x1) ushr 1
            val a = max(x0, mid - (t-1)/2)
            val b = min(x1, a + t - 1)
            if (a-1 >= x0) rec(x0, a-1, z0, z1, depth+1)
            if (b+1 <= x1) rec(b+1, x1, z0, z1, depth+1)
            addColRange(a, b, z0, z1, depth)          // ★ 분리자(두껍게)
        } else {
            val mid = (z0 + z1) ushr 1
            val a = max(z0, mid - (t-1)/2)
            val b = min(z1, a + t - 1)
            if (a-1 >= z0) rec(x0, x1, z0, a-1, depth+1)
            if (b+1 <= z1) rec(x0, x1, b+1, z1, depth+1)
            addRowRange(a, b, x0, x1, depth)          // ★ 분리자(두껍게)
        }
    }

    rec(minCx, maxCx, minCz, maxCz, 0)

    // 누락 보정
    if (order.size != N) {
        val maxLv = level.maxOrNull() ?: 0
        for (u in 0 until N) if (level[u] == -1) { level[u] = maxLv+1; order.add(u) }
    }
    return OrderResult(order.toIntArray(), level)
}

/**
 * 초고속 분할 순서 생성:
 * - 청크 버킷을 2D 그리드로 만들고 (minCx..maxCx, minCz..maxCz)
 * - 2D prefix-sum 으로 빈 영역을 O(1) 판별
 * - 두꺼운 분리자(sepThickness) 지원
 */
fun buildOrderByChunkSeparatorFast(
    g: NavWorldGraph,
    chunkSize: Int = 16,
    sepThickness: Int = 2
): OrderResult {
    val N = g.nodeCount
    if (N == 0) return OrderResult(IntArray(0), IntArray(0))
    require(sepThickness >= 1)

    // 1) 청크 그리드 바운딩 + 버킷팅 (배열 기반)
    var minCx = Int.MAX_VALUE; var maxCx = Int.MIN_VALUE
    var minCz = Int.MAX_VALUE; var maxCz = Int.MIN_VALUE
    val cxArr = IntArray(N); val czArr = IntArray(N)
    for (u in 0 until N) {
        val cx = floorDiv(g.nodeX(u), chunkSize)
        val cz = floorDiv(g.nodeZ(u), chunkSize)
        cxArr[u] = cx; czArr[u] = cz
        if (cx < minCx) minCx = cx; if (cx > maxCx) maxCx = cx
        if (cz < minCz) minCz = cz; if (cz > maxCz) maxCz = cz
    }
    val W = maxCx - minCx + 1
    val H = maxCz - minCz + 1
    val cellCount = W * H

    // 노드 버킷: 각 셀(청크)에 속한 노드 인덱스 목록
    val buckets = Array<MutableList<Int>?>(cellCount) { null }
    // 점유 카운트(셀 당 노드 수)
    val occ = IntArray(cellCount)
    fun idx(gx:Int, gz:Int) = gz * W + gx

    for (u in 0 until N) {
        val gx = cxArr[u] - minCx
        val gz = czArr[u] - minCz
        val id = idx(gx, gz)
        if (buckets[id] == null) buckets[id] = ArrayList()
        buckets[id]!!.add(u)
        occ[id]++
    }

    // 2) 2D prefix sum (ps[(H+1)×(W+1)])
    val ps = IntArray((H + 1) * (W + 1))
    fun psIdx(r:Int, c:Int) = r * (W + 1) + c
    for (gz in 0 until H) {
        var rowSum = 0
        for (gx in 0 until W) {
            rowSum += occ[idx(gx, gz)]
            ps[psIdx(gz + 1, gx + 1)] = ps[psIdx(gz, gx + 1)] + rowSum
        }
    }
    fun regionHasAny(gx0:Int, gx1:Int, gz0:Int, gz1:Int): Boolean {
        if (gx0 > gx1 || gz0 > gz1) return false
        val A = ps[psIdx(gz0,     gx0)]
        val B = ps[psIdx(gz0,     gx1+1)]
        val C = ps[psIdx(gz1+1, gx0)]
        val D = ps[psIdx(gz1+1, gx1+1)]
        // 합(D - B - C + A) > 0 ?
        return (D - B - C + A) > 0
    }

    // 3) 순서/레벨 출력 버퍼
    val order = IntArray(N); var op = 0
    val level = IntArray(N) { -1 }
    fun addCell(gx:Int, gz:Int, depth:Int) {
        val b = buckets[idx(gx, gz)] ?: return
        for (u in b) {
            if (level[u] == -1) {
                level[u] = depth
                order[op++] = u
            }
        }
    }
    fun addColRange(gx0:Int, gx1:Int, gz0:Int, gz1:Int, depth:Int) {
        for (gx in gx0..gx1) {
            // 빠른 스킵: 이 column × [gz0..gz1] 에 노드 없으면 건너뜀
            if (!regionHasAny(gx, gx, gz0, gz1)) continue
            for (gz in gz0..gz1) addCell(gx, gz, depth)
        }
    }
    fun addRowRange(gz0:Int, gz1:Int, gx0:Int, gx1:Int, depth:Int) {
        for (gz in gz0..gz1) {
            if (!regionHasAny(gx0, gx1, gz, gz)) continue
            for (gx in gx0..gx1) addCell(gx, gz, depth)
        }
    }

    // 4) 재귀(그리드 인덱스 공간에서 수행)
    fun rec(gx0:Int, gx1:Int, gz0:Int, gz1:Int, depth:Int) {
        if (!regionHasAny(gx0, gx1, gz0, gz1)) return

        val w = gx1 - gx0 + 1
        val h = gz1 - gz0 + 1
        if (w <= 1 && h <= 1) {
            addCell(gx0, gz0, depth)
            return
        }

        val t = sepThickness
        if (w >= h) {
            val mid = (gx0 + gx1) ushr 1
            val a = max(gx0, mid - (t - 1) / 2)
            val b = min(gx1, a + t - 1)
            // 내부 먼저
            if (a - 1 >= gx0) rec(gx0, a - 1, gz0, gz1, depth + 1)
            if (b + 1 <= gx1) rec(b + 1, gx1, gz0, gz1, depth + 1)
            // 분리자(마지막)
            addColRange(a, b, gz0, gz1, depth)
        } else {
            val mid = (gz0 + gz1) ushr 1
            val a = max(gz0, mid - (t - 1) / 2)
            val b = min(gz1, a + t - 1)
            if (a - 1 >= gz0) rec(gx0, gx1, gz0, a - 1, depth + 1)
            if (b + 1 <= gz1) rec(gx0, gx1, b + 1, gz1, depth + 1)
            addRowRange(a, b, gx0, gx1, depth)
        }
    }

    rec(0, W - 1, 0, H - 1, 0)

    // 누락 보정 (이론상 없어야 하지만 안전장치)
    if (op != N) {
        for (u in 0 until N) if (level[u] == -1) { level[u] = (level.maxOrNull() ?: 0) + 1; order[op++] = u }
    }
    return OrderResult(order.copyOf(op), level)
}