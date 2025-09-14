package kr.arcadia.arcRootFinding.preprocess

import kr.arcadia.arcRootFinding.policy.MovePolicy
import kr.arcadia.arcRootFinding.query.BlockQuery
import kr.arcadia.arcRootFinding.wnm.WNMStore
import java.nio.file.Path
import java.util.UUID

class PreprocessService(
    private val bq: BlockQuery,
    private val store: WNMStore,
    private val policy: MovePolicy
) {
    /**
     * 중심 좌표와 반경으로 전처리 수행. 이전 WNM가 있으면 증분만 갱신.
     */
    fun preprocess(
        outPath: Path,
        worldUUID: UUID,
        centerX: Int,
        centerZ: Int,
        radiusChunks: Int,
        yMin: Int,
        yMax: Int
    ) {
        // 1) 기존 헤더/청크 인덱스 로드(있으면)
        // 2) 스캔 순서: spiral(cx,cz) -> 반경까지
        // 3) 각 청크:
        //    a) mask 계산 -> crc
        //    b) 기존 WNM의 standableCrc와 비교 -> 동일이면 스킵
        //    c) 다르면 buildFromMask로 NavChunk 생성
        // 4) 업데이트 청크 묶음 단위로 store.appendOrPatch()
    }
}