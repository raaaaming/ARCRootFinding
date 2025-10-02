package kr.arcadia.arcPathFinding.wnm.file

import kr.arcadia.arcPathFinding.cch.CCHIndex
import java.io.DataInputStream
import java.io.DataOutputStream
import java.nio.file.Files
import java.nio.file.Path
import java.util.zip.GZIPInputStream
import java.util.zip.GZIPOutputStream

fun writeIndex(path: Path, idx: CCHIndex) {
    Files.newOutputStream(path).use { raw ->
        GZIPOutputStream(raw).use { gz ->
            DataOutputStream(gz).use { out ->
                out.writeInt(0x43434831) // "CCH1"
                val n = idx.rank.size
                val m = idx.upTo.size
                out.writeInt(n); out.writeInt(m)
                // rank
                for (i in 0 until n) out.writeInt(idx.rank[i])
                // upOff
                for (i in 0 .. n) out.writeInt(idx.upOff[i])
                // upTo
                for (i in 0 until m) out.writeInt(idx.upTo[i])
                // upMid
                for (i in 0 until m) out.writeInt(idx.upMid[i])
                // upW
                for (i in 0 until m) out.writeInt(idx.upW[i])
            }
        }
    }
}

fun readIndex(path: Path): CCHIndex {
    Files.newInputStream(path).use { raw ->
        GZIPInputStream(raw).use { gz ->
            DataInputStream(gz).use { `in` ->
                val magic = `in`.readInt()
                require(magic == 0x43434831) { "Bad magic for CCH index" }
                val n = `in`.readInt()
                val m = `in`.readInt()
                val rank = IntArray(n) { `in`.readInt() }
                val upOff = IntArray(n+1) { `in`.readInt() }
                val upTo  = IntArray(m)   { `in`.readInt() }
                val upMid = IntArray(m)   { `in`.readInt() }
                val upW   = IntArray(m)   { `in`.readInt() }
                return CCHIndex(rank, upOff, upTo, upMid, upW)
            }
        }
    }
}