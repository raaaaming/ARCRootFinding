package kr.arcadia.arcPathFinding.query

import kr.arcadia.arcPathFinding.query.impl.BlockQueryImpl
import org.bukkit.World

fun World.getBlockQuery() = BlockQueryImpl(this)