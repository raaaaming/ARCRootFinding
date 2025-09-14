package kr.arcadia.arcPathFinding.separator

import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import kr.arcadia.arcPathFinding.util.IntArrayList
import java.lang.Math.floorDiv

fun buildOrderByChunkSeparator(g: NavWorldGraph, chunkSize: Int = 16): OrderResult {
    val N = g.nodeCount
    if (N == 0) return OrderResult(IntArray(0), IntArray(0))

    data class ChunkKey(val cx:Int, val cz:Int)

    // 1) 노드들을 청크 단위로 버킷팅
    val chunkNodes = HashMap<ChunkKey, IntArrayList>(N / 32 + 1)
    var minCx = Int.MAX_VALUE; var maxCx = Int.MIN_VALUE
    var minCz = Int.MAX_VALUE; var maxCz = Int.MIN_VALUE
    for (u in 0 until N) {
        val cx = floorDiv(g.nodeX(u), chunkSize)
        val cz = floorDiv(g.nodeZ(u), chunkSize)
        val key = ChunkKey(cx, cz)
        val lst = chunkNodes.getOrPut(key){ IntArrayList() }
        lst.add(u)
        if (cx < minCx) minCx = cx
        if (cx > maxCx) maxCx = cx
        if (cz < minCz) minCz = cz
        if (cz > maxCz) maxCz = cz
    }

    // 2) 재귀 도우미들
    fun regionEmpty(x0:Int, x1:Int, z0:Int, z1:Int): Boolean {
        // 영역 내에 단 하나의 점유 청크도 없으면 빈 영역
        for ((k, _) in chunkNodes) {
            if (k.cx in x0..x1 && k.cz in z0..z1) return false
        }
        return true
    }

    val orderList = IntArrayList(N)
    val levelArr = IntArray(N) { -1 }

    fun addNodes(nodes: IntArrayList, depth:Int) {
        var i = 0
        while (i < nodes.size()) {
            val u = nodes[i]
            if (levelArr[u] == -1) { // 안전장치(중복 방지)
                levelArr[u] = depth
                orderList.add(u)
            }
            i++
        }
    }

    fun addAllChunksInRegion(x0:Int, x1:Int, z0:Int, z1:Int, depth:Int) {
        // 영역의 모든 청크 노드를 "현재 순서"로 추가
        for ((k, lst) in chunkNodes) {
            if (k.cx in x0..x1 && k.cz in z0..z1) {
                addNodes(lst, depth)
            }
        }
    }

    fun addSeparatorColumn(cxSep:Int, z0:Int, z1:Int, depth:Int) {
        for (cz in z0..z1) {
            val key = ChunkKey(cxSep, cz)
            chunkNodes[key]?.let { addNodes(it, depth) }
        }
    }
    fun addSeparatorRow(zSep:Int, x0:Int, x1:Int, depth:Int) {
        for (cx in x0..x1) {
            val key = ChunkKey(cx, zSep)
            chunkNodes[key]?.let { addNodes(it, depth) }
        }
    }

    fun rec(x0:Int, x1:Int, z0:Int, z1:Int, depth:Int) {
        if (x0 > x1 || z0 > z1) return
        if (regionEmpty(x0, x1, z0, z1)) return

        val w = x1 - x0 + 1
        val h = z1 - z0 + 1

        // 베이스: 영역이 1청크 이하이면 그냥 싹 담고 끝
        if (w <= 1 && h <= 1) {
            addAllChunksInRegion(x0, x1, z0, z1, depth)
            return
        }

        if (w >= h) {
            // 세로 절단: 중앙 column을 분리자, 양 옆을 재귀
            val mid = (x0 + x1) ushr 1 // 가운데 column
            // 내부(왼쪽/오른쪽)를 먼저
            if (mid - 1 >= x0) rec(x0, mid - 1, z0, z1, depth + 1)
            if (mid + 1 <= x1) rec(mid + 1, x1, z0, z1, depth + 1)
            // 분리자(중앙 column)는 마지막에
            addSeparatorColumn(mid, z0, z1, depth)
        } else {
            // 가로 절단: 중앙 row를 분리자
            val mid = (z0 + z1) ushr 1
            if (mid - 1 >= z0) rec(x0, x1, z0, mid - 1, depth + 1)
            if (mid + 1 <= z1) rec(x0, x1, mid + 1, z1, depth + 1)
            addSeparatorRow(mid, x0, x1, depth)
        }
    }

    // 3) 루트 영역에서 시작
    rec(minCx, maxCx, minCz, maxCz, depth = 0)

    // 4) 결과 배열로 변환 (order: rank->node, level: node->depth)
    val order = orderList.toIntArray()

    // 안전성 체크: 모든 노드가 순서에 포함되어야 함
    if (order.size != N) {
        // 누락이 있으면 남은 노드를 레벨=최대+1로 뒤에 붙여 복구
        val maxLevel = levelArr.maxOrNull() ?: 0
        for (u in 0 until N) {
            if (levelArr[u] == -1) {
                levelArr[u] = maxLevel + 1
                // rank->node 배열의 뒤에 append
                // (간단히 새 배열로 확장)
            }
        }
        val fix = IntArrayList(N)
        fix.apply { // 앞서 만든 순서 먼저
            var i=0; while (i<order.size){ add(order[i]); i++ }
            var u=0; while (u<N){ if (levelArr[u] >= maxLevel+1) add(u); u++ }
        }
        return OrderResult(fix.toIntArray(), levelArr)
    }

    return OrderResult(order, levelArr)
}