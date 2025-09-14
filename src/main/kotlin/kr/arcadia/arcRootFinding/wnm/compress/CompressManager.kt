package kr.arcadia.arcRootFinding.wnm.compress

class CompressManager {
    enum class Codec { RAW, LZ4, ZSTD, GZIP }
    fun compress(codec:Codec, data:ByteArray): ByteArray { /*...*/ TODO() }
    fun decompress(codec:Codec, data:ByteArray): ByteArray { /*...*/ TODO() }
}