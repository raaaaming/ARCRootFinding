package kr.arcadia.arcPathFinding.wnm

data class WNMHeader(
    val version:Int, val policyHash:Int, val worldUUID: java.util.UUID,
    val yMin:Int, val yMax:Int, val createdAt:Long
)