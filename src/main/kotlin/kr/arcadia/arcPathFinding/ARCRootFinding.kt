package kr.arcadia.arcPathFinding

import com.mojang.brigadier.Command
import com.mojang.brigadier.arguments.DoubleArgumentType
import com.mojang.brigadier.builder.LiteralArgumentBuilder
import com.mojang.brigadier.tree.LiteralCommandNode
import io.papermc.paper.command.brigadier.CommandSourceStack
import io.papermc.paper.command.brigadier.Commands
import io.papermc.paper.plugin.lifecycle.event.types.LifecycleEvents
import kr.arcadia.arcPathFinding.cch.contract
import kr.arcadia.arcPathFinding.cch.customizeWeightsPerfect
import kr.arcadia.arcPathFinding.cch.defaultWeightFn
import kr.arcadia.arcPathFinding.chunk.ChunkSigCache
import kr.arcadia.arcPathFinding.core.DynamicCCHPathfinder
import kr.arcadia.arcPathFinding.core.QueryEngine
import kr.arcadia.arcPathFinding.graph.mergeChunks
import kr.arcadia.arcPathFinding.preprocess.PreprocessService
import kr.arcadia.arcPathFinding.separator.buildOrderByChunkSeparator
import kr.arcadia.arcPathFinding.wnm.file.readIndex
import kr.arcadia.arcPathFinding.wnm.file.writeIndex
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

                            // (1) 전처리 (필요 청크 빌드)
                            val pre = PreprocessService(bq, policy)
                            val cache = ChunkSigCache().apply { load(sigPath) }
                            val changedChunks = pre.preprocessRadius(centerX, centerZ, radiusChunks, yMin, yMax, cache)
                            cache.save(sigPath)

                            // (2) 병합 → 전역 그래프
                            val world = mergeChunks(changedChunks /* + 기존 유지 중인 청크들 */)

                            // (3) 순서 생성 → 수축
                            val (order, level) = buildOrderByChunkSeparator(world)
                            val idx = contract(world, order, policy)

                            // (4) 커스터마이즈(가중치 반영)
                            customizeWeightsPerfect(idx, world, ::defaultWeightFn)

                            // (5) I/O (선택)
                            writeIndex(cchPath, idx)
                            val idx2 = readIndex(cchPath) // 검증용

                            // (6) 질의
                            val engine = QueryEngine(world, idx)
                            val path = engine.route(sx,sy,sz, tx,ty,tz) // List<IntArray> (각 [x,y,z])

                            return@executes Command.SINGLE_SUCCESS
                        }
                    )
                )
            )
        ).build()
}
