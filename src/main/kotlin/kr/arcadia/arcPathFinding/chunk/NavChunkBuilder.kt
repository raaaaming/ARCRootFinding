package kr.arcadia.arcPathFinding.chunk

import kr.arcadia.arcPathFinding.util.IntArrayList
import kr.arcadia.arcPathFinding.util.ShortArrayList
import kr.arcadia.arcPathFinding.mask.EdgeTag
import kr.arcadia.arcPathFinding.mask.StandableMask
import kr.arcadia.arcPathFinding.policy.MovePolicy
import kr.arcadia.arcPathFinding.query.BlockQuery
import kotlin.experimental.or

object NavChunkBuilder {

    private val DIR4 = arrayOf(intArrayOf(1, 0), intArrayOf(-1, 0), intArrayOf(0, 1), intArrayOf(0, -1))
    private val DIR8 = arrayOf(
        intArrayOf(1, 0), intArrayOf(-1, 0), intArrayOf(0, 1), intArrayOf(0, -1),
        intArrayOf(1, 1), intArrayOf(1, -1), intArrayOf(-1, 1), intArrayOf(-1, -1)
    )

    /** (bq, pos, yMin, yMax) — 스켈레톤과 동일 */
    fun computeStandableMask(bq: BlockQuery, pos: ChunkPos, yMin: Int, yMax: Int): StandableMask {
        val mask = StandableMask(yMin, yMax)
        val baseX = pos.minX()
        val baseZ = pos.minZ()
        for (lx in 0..15) for (lz in 0..15) {
            val x = baseX + lx;
            val z = baseZ + lz
            for (y in yMin..yMax) {
                val ok =
                    bq.solid(x, y - 1, z) && bq.passable(x, y, z) && bq.passable(x, y + 1, z) && !bq.liquid(x, y - 1, z)
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
        fun key(x: Int, y: Int, z: Int) =
            (x.toLong() shl 42) or ((y.toLong() and 0x3FF) shl 32) or (z.toLong() and 0xFFFFFFFFL)

        val yRange = mask.yRange()
        val baseX = pos.minX();
        val baseZ = pos.minZ()
        for (lx in 0..15) for (lz in 0..15) {
            val x = baseX + lx;
            val z = baseZ + lz
            for (y in yRange) {
                if (!mask.getLocal(lx, y, lz)) continue
                val k = key(x, y, z)
                if (indexOf[k] == null) {
                    indexOf[k] = coords.size() / 3; coords.add(x); coords.add(y); coords.add(z)
                }
            }
        }
        val nodeCount = coords.size() / 3
        val crc = mask.crc32()
        if (nodeCount == 0) {
            return NavChunk(
                pos, IntArray(0), IntArray(1) { 0 }, IntArray(0), ShortArray(0),
                /*gateN*/IntArray(0), /*gateS*/IntArray(0), /*gateW*/IntArray(0), /*gateE*/IntArray(0),
                crc
            )
        }

        // --- 2) 간선 생성 (CSR 임시 버퍼) ---  ← 이 줄부터 기존 코드를 전부 교체
        val nbr = Array(nodeCount) { IntArrayList() }
        val tag = Array(nodeCount) { ShortArrayList() }
        val dirs = if (policy.allowDiag) DIR8 else DIR4

        val yMin = mask.yRange().first;
        val yMax = mask.yRange().last
        val yLen = yMax - yMin + 1

// 로컬 좌표 → 로컬 노드 id 매핑 (O(1) 조회)
        val idAt = Array(16) { Array(yLen) { IntArray(16) { -1 } } }
        run {
            var u = 0
            while (u < nodeCount) {
                val wx = coords[u * 3]
                val wy = coords[u * 3 + 1]
                val wz = coords[u * 3 + 2]
                val lx = wx - baseX
                val lz = wz - baseZ
                idAt[lx][wy - yMin][lz] = u
                u++
            }
        }

        // 마스크 기반 standable 확인
        fun isStandable(lx: Int, y: Int, lz: Int): Boolean =
            (lx in 0..15) && (lz in 0..15) && (y in yMin..yMax) && mask.getLocal(lx, y, lz)

        // “대각 코너클립” 방지: 2블럭(몸/머리) 모두 고려
        fun cornerClippedTwoTall(wx: Int, wy: Int, wz: Int, dx: Int, dz: Int): Boolean {
            if (!policy.forbidCornerClip || dx == 0 || dz == 0) return false
            fun blocked(x: Int, y: Int, z: Int): Boolean =
                !(bq.passable(x, y, z) && bq.passable(x, y + 1, z))
            return blocked(wx + dx, wy, wz) && blocked(wx, wy, wz + dz)
        }

        // 낙하 k칸 시, 수직 경로(머리/몸) 클리어 확인
        fun verticalAirClear(x: Int, yTop: Int, z: Int, k: Int): Boolean {
            var i = 1
            while (i <= k) {
                if (!bq.passable(x, yTop - i, z)) return false
                if (!bq.passable(x, yTop - i + 1, z)) return false
                i++
            }
            return true
        }

        // 도착할 베스트 y: 같은 높이 ▶ stepUp까지 상승 ▶ maxDrop까지 하강
        fun bestTargetY(nlx: Int, nlz: Int, y: Int, stepUp: Int, maxDrop: Int): Int? {
            if (isStandable(nlx, y, nlz)) return y
            var d = 1
            while (d <= stepUp) {
                if (isStandable(nlx, y + d, nlz)) return y + d; d++
            }
            d = 1
            while (d <= maxDrop) {
                if (isStandable(nlx, y - d, nlz)) return y - d; d++
            }
            return null
        }

        var uLocal = 0
        while (uLocal < nodeCount) {
            val wx = coords[uLocal * 3]
            val wy = coords[uLocal * 3 + 1]
            val wz = coords[uLocal * 3 + 2]
            val lx = wx - baseX
            val lz = wz - baseZ

            // 사다리 수직 이동(옵션)
            if (bq.ladder(wx, wy, wz)) {
                val upY = wy + 1
                val dnY = wy - 1
                if (upY <= yMax && isStandable(lx, upY, lz)) {
                    val v = idAt[lx][upY - yMin][lz]
                    if (v >= 0) {
                        nbr[uLocal].add(v); tag[uLocal].add(EdgeTag.LADDER)
                    }
                }
                if (dnY >= yMin && isStandable(lx, dnY, lz)) {
                    val v = idAt[lx][dnY - yMin][lz]
                    if (v >= 0) {
                        nbr[uLocal].add(v); tag[uLocal].add(EdgeTag.LADDER)
                    }
                }
            }

            // 4/8 방향 이웃
            for (d in dirs) {
                val dx = d[0];
                val dz = d[1]
                val nlx = lx + dx
                val nlz = lz + dz
                // 인청크 내에서만 처리 (경계 넘는 건 게이트로)
                if (nlx !in 0..15 || nlz !in 0..15) continue

                // 대각 코너클립 체크(두 칸 키)
                if (cornerClippedTwoTall(wx, wy, wz, dx, dz)) continue

                val ty = bestTargetY(nlx, nlz, wy, policy.stepUp, policy.maxDrop) ?: continue

                // 낙하라면 수직 경로 확인
                val drop = wy - ty
                if (drop > 0 && !verticalAirClear(wx + dx, wy, wz + dz, drop)) continue

                val vLocal = idAt[nlx][ty - yMin][nlz]
                if (vLocal >= 0) {
                    var t: Short = 0
                    if (kotlin.math.abs(dx) + kotlin.math.abs(dz) == 2) t = (t or EdgeTag.DIAG)
                    if (ty > wy) t = (t or EdgeTag.STEP_UP)
                    if (ty < wy) {
                        t = t or when (drop) {
                            1 -> EdgeTag.DROP1
                            2 -> EdgeTag.DROP2
                            else -> EdgeTag.DROP3
                        }
                    }
                    nbr[uLocal].add(vLocal)
                    tag[uLocal].add(t)
                }
            }
            uLocal++
        }

        // --- 3) CSR 압축 ---
        val off = IntArray(nodeCount + 1);
        var m = 0
        for (i in 0 until nodeCount) {
            off[i] = m; m += nbr[i].size()
        }
        off[nodeCount] = m
        val to = IntArray(m);
        val tg = ShortArray(m)
        for (i in 0 until nodeCount) {
            val base = off[i];
            val lst = nbr[i];
            val tgs = tag[i]
            var j = 0; while (j < lst.size()) {
                to[base + j] = lst[j]; tg[base + j] = tgs[j]; j++
            }
        }

        // --- 4) 게이트 수집 (gateN/S/W/E — 스켈레톤 네이밍) ---
        val gateN = IntArrayList();
        val gateS = IntArrayList()
        val gateW = IntArrayList();
        val gateE = IntArrayList()
        val minX = pos.minX();
        val maxX = pos.maxX()
        val minZ = pos.minZ();
        val maxZ = pos.maxZ()
        for (i in 0 until nodeCount) {
            val x = coords[i * 3];
            val z = coords[i * 3 + 2]
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