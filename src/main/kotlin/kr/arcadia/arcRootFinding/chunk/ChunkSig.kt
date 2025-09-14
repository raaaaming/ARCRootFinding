package kr.arcadia.arcRootFinding.chunk

data class ChunkSig(
    val pos:ChunkPos,
    val yMin:Int,
    val yMax:Int,
    val policyHash:Int,
    val standableCrc:Int
)