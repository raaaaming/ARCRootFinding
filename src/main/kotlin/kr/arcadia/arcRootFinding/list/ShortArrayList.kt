package kr.arcadia.arcRootFinding.list

import kotlin.math.max

class ShortArrayList(cap:Int = 8) {
    private var a = ShortArray(cap)
    private var n = 0
    fun add(v:Short){ if (n==a.size) a = a.copyOf(max(8, a.size*2)); a[n++] = v }
    fun size() = n
    operator fun get(i:Int) = a[i]
    fun toShortArray() = a.copyOf(n)
}