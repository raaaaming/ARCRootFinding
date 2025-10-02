package kr.arcadia.arcPathFinding.query.impl

import kr.arcadia.arcPathFinding.query.BlockQuery
import org.bukkit.Material
import org.bukkit.World

class BlockQueryImpl(private val world: World) : BlockQuery {
    private fun type(x:Int,y:Int,z:Int) = world.getBlockAt(x,y,z).type

    override fun solid(x:Int,y:Int,z:Int): Boolean {
        val t = type(x,y,z)
        // Paper의 isSolid는 대체로 OK. 필요시 화로/울타리 등 세밀 조정.
        return t.isSolid
    }
    override fun passable(x:Int,y:Int,z:Int): Boolean {
        val t = type(x,y,z)
        return t.isAir || !t.isSolid // 간단 기준. 필요시 문/울타리 규칙 보강.
    }
    override fun liquid(x:Int,y:Int,z:Int): Boolean {
        return when (type(x,y,z)) {
            Material.LAVA, Material.WATER -> true
            else -> false
        }
    }
    override fun ladder(x:Int,y:Int,z:Int): Boolean {
        return type(x,y,z) == Material.LADDER
    }
}