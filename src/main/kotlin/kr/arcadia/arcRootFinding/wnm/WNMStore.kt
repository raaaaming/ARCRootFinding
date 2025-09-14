package kr.arcadia.arcRootFinding.wnm

import kr.arcadia.arcRootFinding.chunk.ChunkPos
import kr.arcadia.arcRootFinding.chunk.NavChunk
import kr.arcadia.arcRootFinding.wnm.compress.CompressManager
import java.nio.file.Path

class WNMStore(private val compress: CompressManager) {
    fun loadHeader(path: Path): WNMHeader { /*...*/ TODO() }
    fun loadChunkIndex(path: Path): Map<ChunkPos,Long> { /* entry -> file offset */ TODO() }
    fun writeNew(path: Path, header: WNMHeader, chunks: List<NavChunk>) { /* 섹션별 압축/쓰기 */ TODO() }
    fun appendOrPatch(path: Path, updated: List<NavChunk>) { /* 델타 반영 */ TODO() }
}