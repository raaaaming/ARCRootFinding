package kr.arcadia.arcRootFinding.chunk

data class ChunkPos(
    val cx:Int,
    val cz:Int
) {
    fun minX() = cx shl 4
    fun minZ() = cz shl 4
    fun maxX() = (cx shl 4) + 15
    fun maxZ() = (cz shl 4) + 15
}