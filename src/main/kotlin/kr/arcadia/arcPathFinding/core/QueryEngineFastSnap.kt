package kr.arcadia.arcPathFinding.core

import kr.arcadia.arcPathFinding.cch.CCHIndex
import kr.arcadia.arcPathFinding.chunk.ChunkSpatialIndex
import kr.arcadia.arcPathFinding.graph.NavWorldGraph

class QueryEngineFastSnap(
    override val g: NavWorldGraph,
    override val idx: CCHIndex,
    private val snap: ChunkSpatialIndex
): QueryEngine(g, idx) { // 기존 QueryEngine 상속이 아니라면 구성(컴포지션)으로 써도 OK
    fun routeFast(sx:Int, sy:Int, sz:Int, tx:Int, ty:Int, tz:Int) =
        super.routeWithSnap({ x,y,z -> snap.nearestNode(x,y,z) }, sx,sy,sz, tx,ty,tz)
}