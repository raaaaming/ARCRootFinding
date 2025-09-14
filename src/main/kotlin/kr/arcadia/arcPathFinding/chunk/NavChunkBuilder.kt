package kr.arcadia.arcPathFinding.chunk

import kr.arcadia.arcPathFinding.util.IntArrayList
import kr.arcadia.arcPathFinding.util.ShortArrayList
import kr.arcadia.arcPathFinding.mask.EdgeTag
import kr.arcadia.arcPathFinding.mask.StandableMask
import kr.arcadia.arcPathFinding.policy.MovePolicy
import kr.arcadia.arcPathFinding.query.BlockQuery
import kotlin.experimental.or

object NavChunkBuilder {

    private val DIR4 = arrayOf(intArrayOf(1,0), intArrayOf(-1,0), intArrayOf(0,1), intArrayOf(0,-1))
    private val DIR8 = arrayOf(
        intArrayOf( 1, 0), intArrayOf(-1, 0), intArrayOf( 0, 1), intArrayOf( 0,-1),
        intArrayOf( 1, 1), intArrayOf( 1,-1), intArrayOf(-1, 1), intArrayOf(-1,-1)
    )

    /** (bq, pos, yMin, yMax) — 스켈레톤과 동일 */
    fun computeStandableMask(bq: BlockQuery, pos: ChunkPos, yMin: Int, yMax: Int): StandableMask {
        val mask = StandableMask(yMin, yMax)
        val baseX = pos.minX()
        val baseZ = pos.minZ()
        for (lx in 0..15) for (lz in 0..15) {
            val x = baseX + lx; val z = baseZ + lz
            for (y in yMin..yMax) {
                val ok = bq.solid(x, y-1, z) && bq.passable(x, y, z) && bq.passable(x, y+1, z) && !bq.liquid(x, y-1, z)
                if (ok) mask.setLocal(lx, y, lz, true)
            }
        }
        return mask
    }

    /** (bq, pos, mask, policy) — 스켈레톤과 동일 */
    fun buildFromMask(bq: BlockQuery, pos: ChunkPos, mask: StandableMask, policy: MovePolicy): NavChunk {
        // --- 1) 노드 수집 (워커블만) ---
        val coords = IntArrayList(1024)
        val indexOf = HashMap<Long, Int>(1024)
        fun key(x:Int,y:Int,z:Int) = (x.toLong() shl 42) or ((y.toLong() and 0x3FF) shl 32) or (z.toLong() and 0xFFFFFFFFL)

        val yRange = mask.yRange()
        val baseX = pos.minX(); val baseZ = pos.minZ()
        for (lx in 0..15) for (lz in 0..15) {
            val x = baseX + lx; val z = baseZ + lz
            for (y in yRange) {
                if (!mask.getLocal(lx, y, lz)) continue
                val k = key(x,y,z)
                if (indexOf[k] == null) { indexOf[k] = coords.size() / 3; coords.add(x); coords.add(y); coords.add(z) }
            }
        }
        val nodeCount = coords.size() / 3
        val crc = mask.crc32()
        if (nodeCount == 0) {
            return NavChunk(pos, IntArray(0), IntArray(1){0}, IntArray(0), ShortArray(0),
                /*gateN*/IntArray(0), /*gateS*/IntArray(0), /*gateW*/IntArray(0), /*gateE*/IntArray(0),
                crc)
        }

        // --- 2) 간선 생성 (CSR 임시 버퍼) ---
        val nbr = Array(nodeCount){ IntArrayList() }
        val tag = Array(nodeCount){ ShortArrayList() }
        val dirs = if (policy.allowDiag) DIR8 else DIR4
        fun hasNode(x:Int,y:Int,z:Int): Int? = indexOf[key(x,y,z)]
        fun cornerBlocked(px:Int, py:Int, pz:Int, dx:Int, dz:Int): Boolean {
            if (!policy.allowDiag || !policy.forbidCornerClip || kotlin.math.abs(dx)+kotlin.math.abs(dz)!=2) return false
            val b1 = bq.solid(px + dx, py, pz)
            val b2 = bq.solid(px, py, pz + dz)
            return b1 && b2
        }
        fun verticalAirClear(x:Int, yTop:Int, z:Int, k:Int): Boolean {
            var i = 1
            while (i <= k) {
                if (!bq.passable(x, yTop - i, z)) return false
                if (!bq.passable(x, yTop - i + 1, z)) return false
                i++
            }
            return true
        }

        for (u in 0 until nodeCount) {
            val px = coords[u*3]; val py = coords[u*3+1]; val pz = coords[u*3+2]

            // 사다리 상하 (옵션)
            if (bq.ladder(px, py, pz)) {
                hasNode(px, py+1, pz)?.let { v -> nbr[u].add(v); tag[u].add(EdgeTag.LADDER) }
                hasNode(px, py-1, pz)?.let { v -> nbr[u].add(v); tag[u].add(EdgeTag.LADDER) }
            }

            for (d in dirs) {
                val dx = d[0]; val dz = d[1]

                // 수평
                run {
                    val qx = px + dx; val qy = py; val qz = pz + dz
                    if (!cornerBlocked(px, py, pz, dx, dz)) {
                        hasNode(qx,qy,qz)?.let { v ->
                            nbr[u].add(v)
                            var t: Short = 0
                            if (kotlin.math.abs(dx)+kotlin.math.abs(dz)==2) t = (t or EdgeTag.DIAG)
                            tag[u].add(t)
                        }
                    }
                }
                // +1 상승
                if (policy.stepUp >= 1) {
                    val qx = px + dx; val qy = py + 1; val qz = pz + dz
                    val head1 = bq.passable(qx, py+1, qz)
                    val head2 = bq.passable(qx, py+2, qz)
                    if (head1 && head2) {
                        hasNode(qx,qy,qz)?.let { v ->
                            nbr[u].add(v)
                            var t: Short = EdgeTag.STEP_UP
                            if (kotlin.math.abs(dx)+kotlin.math.abs(dz)==2) t = (t or EdgeTag.DIAG)
                            tag[u].add(t)
                        }
                    }
                }
                // 하강 1..maxDrop
                var k = 1
                while (k <= policy.maxDrop) {
                    val qx = px + dx; val qy = py - k; val qz = pz + dz
                    val v = hasNode(qx,qy,qz)
                    if (v != null && verticalAirClear(qx, py, qz, k)) {
                        nbr[u].add(v)
                        val t: Short = when (k) { 1 -> EdgeTag.DROP1; 2 -> EdgeTag.DROP2; else -> EdgeTag.DROP3 }
                        tag[u].add(t)
                    }
                    k++
                }
            }
        }

        // --- 3) CSR 압축 ---
        val off = IntArray(nodeCount + 1); var m = 0
        for (i in 0 until nodeCount) { off[i] = m; m += nbr[i].size() }
        off[nodeCount] = m
        val to = IntArray(m); val tg = ShortArray(m)
        for (i in 0 until nodeCount) {
            val base = off[i]; val lst = nbr[i]; val tgs = tag[i]
            var j = 0; while (j < lst.size()) { to[base+j] = lst[j]; tg[base+j] = tgs[j]; j++ }
        }

        // --- 4) 게이트 수집 (gateN/S/W/E — 스켈레톤 네이밍) ---
        val gateN = IntArrayList(); val gateS = IntArrayList()
        val gateW = IntArrayList(); val gateE = IntArrayList()
        val minX = pos.minX(); val maxX = pos.maxX()
        val minZ = pos.minZ(); val maxZ = pos.maxZ()
        for (i in 0 until nodeCount) {
            val x = coords[i*3]; val z = coords[i*3+2]
            when {
                z == minZ -> gateN.add(i)
                z == maxZ -> gateS.add(i)
                x == minX -> gateW.add(i)
                x == maxX -> gateE.add(i)
            }
        }

        return NavChunk(
            pos = pos,
            coords = coords.toIntArray(),
            adjOff = off,
            adjTo = to,
            adjTag = tg,
            gateN = gateN.toIntArray(),
            gateS = gateS.toIntArray(),
            gateW = gateW.toIntArray(),
            gateE = gateE.toIntArray(),
            standableCrc = crc
        )
    }
}