package kr.arcadia.arcRootFinding

import com.mojang.brigadier.Command
import com.mojang.brigadier.arguments.DoubleArgumentType
import com.mojang.brigadier.builder.LiteralArgumentBuilder
import com.mojang.brigadier.tree.LiteralCommandNode
import io.papermc.paper.command.brigadier.CommandSourceStack
import io.papermc.paper.command.brigadier.Commands
import io.papermc.paper.plugin.lifecycle.event.types.LifecycleEvents
import kr.arcadia.arcRootFinding.core.DynamicCCHPathfinder
import kr.arcadia.core.bukkit.ARCBukkitPlugin
import org.bukkit.Location
import org.bukkit.entity.Player
import org.bukkit.scheduler.BukkitRunnable

class ARCRootFinding : ARCBukkitPlugin() {

    private lateinit var pathfinder: DynamicCCHPathfinder

    override fun onPostLoad() {
        pathfinder = DynamicCCHPathfinder(this)
    }

    override fun onPreEnable() {
        lifecycleManager.registerEventHandler(LifecycleEvents.COMMANDS) { command ->
            command.registrar().register(buildCommand)
        }
    }

    val buildCommand: LiteralCommandNode<CommandSourceStack> = LiteralArgumentBuilder.literal<CommandSourceStack>("arcrootfinding")
        .then(Commands.literal("test")
            .then(Commands.argument("destination's X", DoubleArgumentType.doubleArg())
                .then(Commands.argument("destination's Y", DoubleArgumentType.doubleArg())
                    .then(Commands.argument("destination's Z", DoubleArgumentType.doubleArg())
                        .executes { ctx ->
                            val x = DoubleArgumentType.getDouble(ctx, "destination's X")
                            val y = DoubleArgumentType.getDouble(ctx, "destination's Y")
                            val z = DoubleArgumentType.getDouble(ctx, "destination's Z")
                            val start = ctx.source.location
                            val world = start.world
                            val destination = Location(world, x, y, z)
                            val sender = (ctx.source.sender as Player)

                            sender.sendMessage("경로 탐색을 시작합니다...")

                            pathfinder.findPathAsync(start, destination).thenAccept { path ->
                                if (path != null) {
                                    sender.sendMessage("§a경로를 찾았습니다! 길이: ${path.size} 블록")

                                    // 경로를 파티클로 표시 (선택사항)
                                    object : BukkitRunnable() {
                                        override fun run() {
                                            path.forEach { loc ->
                                                world.spawnParticle(
                                                    org.bukkit.Particle.FLAME,
                                                    loc.add(0.0, 1.0, 0.0),
                                                    1
                                                )
                                            }
                                        }
                                    }.runTaskLater(this, 0L)

                                } else {
                                    sender.sendMessage("§c경로를 찾을 수 없습니다.")
                                }
                            }

                            return@executes Command.SINGLE_SUCCESS
                        }
                    )
                )
            )
        ).build()
}
