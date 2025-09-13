package kr.arcadia.arcRootFinding

import com.mojang.brigadier.Command
import com.mojang.brigadier.arguments.DoubleArgumentType
import io.papermc.paper.command.brigadier.Commands
import kr.arcadia.core.bukkit.ARCBukkitPlugin
import org.bukkit.FluidCollisionMode
import org.bukkit.Location
import org.bukkit.entity.Player

class ARCRootFinding : ARCBukkitPlugin() {

    override fun onPostEnable() {

    }

    override fun onPreDisable() {
    }

    val buildCommand = Commands.literal("arcrootfinding")
        .then(Commands.literal("test")
            .then(Commands.argument("destination's X", DoubleArgumentType.doubleArg())
                .then(Commands.argument("destination's Y", DoubleArgumentType.doubleArg())
                    .then(Commands.argument("destination's Z", DoubleArgumentType.doubleArg())
                        .executes { ctx ->
                            val x = DoubleArgumentType.getDouble(ctx, "destination's X")
                            val y = DoubleArgumentType.getDouble(ctx, "destination's Y")
                            val z = DoubleArgumentType.getDouble(ctx, "destination's Z")
                            val start = ctx.source.location
                            val destination = Location(ctx.source.location.world, x, y, z)
                            val sender = (ctx.source.sender as Player)
                            var goVec = destination.subtract(start).toVector().normalize()
                            val rayTrace = sender.rayTraceBlocks(3.0, FluidCollisionMode.SOURCE_ONLY)
                            if(rayTrace?.hitBlock == null) {
                                sender.sendMessage("No block was hit.")
                                return@executes Command.SINGLE_SUCCESS
                            }
                            val blockVec = rayTrace.hitPosition.subtract(start.toVector()).normalize()
                            goVec = blockVec.subtract(goVec).normalize() //TODO how to rotate my goVec for avoiding this obstacle?
                            return@executes Command.SINGLE_SUCCESS
                        }
                    )
                )
            )
        )
}
