package kr.arcadia.arcPathFinding.graph

class NavWorldGraph(
    val nodeCount: Int,
    val xyzt: IntArray,      // [x,y,z] × N
    val chunkOf: IntArray,   // node -> chunkIndex
    val csrOff: IntArray,    // CSR offsets (size = N+1)
    val csrTo: IntArray      // CSR neighbors (global node id)
) {
    fun nodeX(i:Int) = xyzt[i*3]
    fun nodeY(i:Int) = xyzt[i*3+1]
    fun nodeZ(i:Int) = xyzt[i*3+2]
}