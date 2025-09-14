package kr.arcadia.arcRootFinding.chunk

data class NavChunk(
    val pos:ChunkPos,
    val coords:IntArray,           // nodeLocal -> [x,y,z]×N
    val adjOff:IntArray,           // CSR off
    val adjTo:IntArray,            // CSR to (local)
    val adjTag:ShortArray,         // edge flags
    val gateN:IntArray,
    val gateS:IntArray,
    val gateW:IntArray,
    val gateE:IntArray,
    val standableCrc:Int           // 증분 판별용
) {
    val nodeCount
        get()= coords.size/3
}