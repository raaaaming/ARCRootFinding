package kr.arcadia.arcRootFinding.mask

object EdgeTag {
    const val DIAG: Short = 1
    const val STEP_UP: Short = (1 shl 1).toShort()
    const val DROP1: Short = (1 shl 2).toShort()
    const val DROP2: Short = (1 shl 3).toShort()
    const val DROP3: Short = (1 shl 4).toShort()
    const val LADDER: Short = (1 shl 5).toShort()
}