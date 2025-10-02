package kr.arcadia.arcPathFinding.chunk

data class ChunkSig(
    val cx:Int,
    val cz:Int,
    val yMin:Int,
    val yMax:Int,
    val policyHash:Int,
    val standableCrc:Int
)