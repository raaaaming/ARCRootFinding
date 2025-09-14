package kr.arcadia.arcRootFinding.mask

import kr.arcadia.arcRootFinding.chunk.ChunkPos

class StandableMask(private val yMin: Int, yMax: Int) {
    private val height = (yMax - yMin + 1)
    private val bits = java.util.BitSet(16 * 16 * height)
    private fun idx(lx:Int, ly:Int, lz:Int) = ((ly * 16 + lz) * 16 + lx)

    fun getLocal(lx:Int, y:Int, lz:Int): Boolean {
        val ly = y - yMin
        return (lx in 0..15) && (lz in 0..15) && (ly in 0 until height) && bits.get(idx(lx, ly, lz))
    }
    fun setLocal(lx:Int, y:Int, lz:Int, v:Boolean) {
        val ly = y - yMin
        if ((lx !in 0..15) || (lz !in 0..15) || (ly !in 0 until height)) return
        bits.set(idx(lx, ly, lz), v)
    }
    fun getWorld(pos: ChunkPos, x:Int, y:Int, z:Int) = getLocal(x - pos.minX(), y, z - pos.minZ())
    fun setWorld(pos:ChunkPos, x:Int, y:Int, z:Int, v:Boolean) = setLocal(x - pos.minX(), y, z - pos.minZ(), v)

    fun crc32(): Int { val crc = java.util.zip.CRC32(); crc.update(bits.toByteArray()); return crc.value.toInt() }

    /** buildFromMask에서 y 범위를 정확히 돌기 위한 공개 헬퍼 */
    fun yRange(): IntRange = yMin..< yMin + height
}