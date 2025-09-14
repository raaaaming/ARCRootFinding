package kr.arcadia.arcPathFinding.policy

import java.util.Objects

data class MovePolicy(
    val allowDiag:Boolean = true,
    val maxDrop:Int = 3,
    val stepUp:Int = 1,
    val forbidCornerClip:Boolean = true
) {
    fun hash():Int = Objects.hash(allowDiag,maxDrop,stepUp,forbidCornerClip)
}