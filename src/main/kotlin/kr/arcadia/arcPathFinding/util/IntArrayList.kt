package kr.arcadia.arcPathFinding.util

import kotlin.math.max

class IntArrayList(cap:Int = 8) {
    private var a = IntArray(cap)
    private var n = 0
    fun add(v:Int){ if (n==a.size) a = a.copyOf(max(8, a.size*2)); a[n++] = v }
    fun size() = n
    operator fun get(i:Int) = a[i]
    fun toIntArray() = a.copyOf(n)
}