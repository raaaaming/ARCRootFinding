package kr.arcadia.arcPathFinding.query

interface BlockQuery {
    fun solid(x:Int,y:Int,z:Int):Boolean
    fun passable(x:Int,y:Int,z:Int):Boolean
    fun liquid(x:Int,y:Int,z:Int):Boolean
    fun ladder(x:Int,y:Int,z:Int):Boolean
}