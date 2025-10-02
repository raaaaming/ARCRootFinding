package kr.arcadia.arcPathFinding.policy

data class WeightPolicy(
    val base:Int = 10, val diagExtra:Int = 4, val stepUp:Int = 4,
    val fallPer:Int = 1, val ladderBias:Int = -2, val nightMultiplier:Double = 1.0
)