package kr.arcadia.arcPathFinding.util

import kotlin.math.max

class PairArrayList(cap:Int = 8) {
    private var to = IntArray(cap)
    private var mid = IntArray(cap)
    private var n = 0
    fun add(t:Int, m:Int){
        if (n==to.size){
            val ncap = max(8, to.size*2)
            to = to.copyOf(ncap); mid = mid.copyOf(ncap)
        }
        to[n] = t; mid[n] = m; n++
    }
    fun size() = n
    fun getTo(i:Int) = to[i]
    fun getMid(i:Int) = mid[i]
    fun toArrays(): Pair<IntArray, IntArray> = to.copyOf(n) to mid.copyOf(n)
}