package kr.arcadia.arcPathFinding

import com.mojang.brigadier.Command
import com.mojang.brigadier.arguments.DoubleArgumentType
import com.mojang.brigadier.arguments.IntegerArgumentType
import com.mojang.brigadier.builder.LiteralArgumentBuilder
import com.mojang.brigadier.tree.LiteralCommandNode
import io.papermc.paper.command.brigadier.CommandSourceStack
import io.papermc.paper.command.brigadier.Commands
import io.papermc.paper.plugin.lifecycle.event.types.LifecycleEvents
import kr.arcadia.arcPathFinding.cch.CCHIndex
import kr.arcadia.arcPathFinding.cch.contractUltra
import kr.arcadia.arcPathFinding.cch.customizeUpperTrianglePass
import kr.arcadia.arcPathFinding.cch.customizeWeightsPerfectFast
import kr.arcadia.arcPathFinding.chunk.ChunkSigCache
import kr.arcadia.arcPathFinding.chunk.ChunkSpatialIndex
import kr.arcadia.arcPathFinding.chunk.NavChunk
import kr.arcadia.arcPathFinding.core.QueryEngine
import kr.arcadia.arcPathFinding.graph.NavWorldGraph
import kr.arcadia.arcPathFinding.graph.mergeChunks
import kr.arcadia.arcPathFinding.policy.MovePolicy
import kr.arcadia.arcPathFinding.policy.WeightPolicy
import kr.arcadia.arcPathFinding.policy.makeAttrWeightFn
import kr.arcadia.arcPathFinding.policy.node.buildNodeAttrs
import kr.arcadia.arcPathFinding.preprocess.PreprocessService
import kr.arcadia.arcPathFinding.query.getBlockQuery
import kr.arcadia.arcPathFinding.separator.buildOrderByChunkSeparatorFast
import kr.arcadia.arcPathFinding.wnm.WNMHeader
import kr.arcadia.arcPathFinding.wnm.WNMStore
import kr.arcadia.arcPathFinding.wnm.file.readIndex
import kr.arcadia.arcPathFinding.wnm.file.writeIndex
import kr.arcadia.core.bukkit.ARCBukkitPlugin
import org.bukkit.Location
import org.bukkit.Particle
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import org.bukkit.scheduler.BukkitTask
import org.bukkit.World
import java.util.UUID

interface ARCPathFindingAPI {
    fun getPath(origin: Location, destination: Location): List<Location>
    fun getPath(destination: Location): List<Location>
}

class ARCPathFinding : ARCBukkitPlugin() {

    companion object {
        @Volatile
        private var instance: ARCPathFinding? = null

        @JvmStatic
        fun getAPI(): ARCPathFindingAPI {
            val plugin = instance ?: throw IllegalStateException("ARCPathFinding is not enabled yet")
            return plugin.api
        }
    }

    private lateinit var worldGraph: NavWorldGraph
    private lateinit var cchIndex: CCHIndex
    private lateinit var engine: QueryEngine
    private lateinit var snap: ChunkSpatialIndex
    private lateinit var pathWorld: World
    private val activePathTasks = mutableMapOf<UUID, BukkitTask>()

    private val dataDir by lazy { dataFolder.toPath() }
    private val wnmPath by lazy { dataDir.resolve("map.wnm") }
    private val cchPath by lazy { dataDir.resolve("map.cch.gz") }
    private val sigPath by lazy { dataDir.resolve("map.sig.gz") }

    private val api: ARCPathFindingAPI = object : ARCPathFindingAPI {
        override fun getPath(origin: Location, destination: Location): List<Location> {
            return computePath(origin, destination)
        }

        override fun getPath(destination: Location): List<Location> {
            val world = destination.world ?: throw IllegalArgumentException("Destination must have an associated world")
            return computePath(world.spawnLocation, destination)
        }
    }

    override fun onPreEnable() {
        instance = this
        lifecycleManager.registerEventHandler(LifecycleEvents.COMMANDS) { command ->
            command.registrar().register(buildCommand)
        }
    }

    override fun onPostEnable() {
        if (!dataFolder.exists()) dataFolder.mkdirs()

        val w = server.worlds.first()               // 대상 월드 선택(예: overworld)
        pathWorld = w
        val bq = w.getBlockQuery()
        val policy = MovePolicy(allowDiag = true, maxDrop = 3, stepUp = 1, forbidCornerClip = true)

        // ---- M6 증분 전처리 (중심/반경/높이 범위는 서버 상황에 맞게) ----
        val pre = PreprocessService(bq, policy)
        val sigCache = ChunkSigCache().apply { load(sigPath) }
        val center = w.spawnLocation
        val yMin = 40; val yMax = 140
        val changed: List<NavChunk> = pre.preprocessRadius(center.blockX, center.blockZ, radiusChunks = 50, yMin, yMax, sigCache)
        sigCache.save(sigPath)

        // ---- M5 WNM 저장소: 기존 + 변경분 병합 저장 ----
        val store = WNMStore()
        val header = WNMHeader(
            version = 1,
            policyHash = policy.hash(),
            worldUUID = UUID(0L,0L),   // 필요시 실제 world UID 사용
            yMin = yMin, yMax = yMax,
            createdAt = System.currentTimeMillis()
        )
        store.appendOrPatch(wnmPath, header, changed)

        // ---- 전체 로드 → 병합 → CCH 수축 → 커스터마이즈 ----
        val (_, chunks) = store.readAll(wnmPath)
        worldGraph = mergeChunks(chunks, policy)

        val (order, _) = buildOrderByChunkSeparatorFast(worldGraph, sepThickness = 4) // ★ 두께 2~3 추천
        val contractedIndex = contractUltra(worldGraph, order, policy)                        // ★ 빠른 수축

        println("병합 → CCH 수축") //클리어

        val attrs = buildNodeAttrs(worldGraph, bq)
        val weightFn = makeAttrWeightFn(attrs, WeightPolicy(nightMultiplier = 1.0))
        customizeWeightsPerfectFast(contractedIndex, worldGraph, weightFn)
        customizeUpperTrianglePass(contractedIndex) // (선택) 1회 추가 패스


        // CCH 인덱스 저장/로드(선택)
        writeIndex(cchPath, contractedIndex)
        cchIndex = readIndex(cchPath)

        println("CCH 인덱스 저장/로드") //클리어


        // ---- 빠른 스냅 인덱스 ----
        snap = ChunkSpatialIndex(worldGraph)

        // ---- 질의 엔진 ----
        engine = QueryEngine(worldGraph, cchIndex)

        println("질의 엔진") //클리어

    }

    override fun onPreDisable() {
        activePathTasks.values.forEach { it.cancel() }
        activePathTasks.clear()
        instance = null
    }

    private fun ensureReady() {
        check(::engine.isInitialized && ::snap.isInitialized) { "ARCPathFinding has not finished loading yet" }
    }

    private fun computePath(origin: Location, destination: Location): List<Location> {
        ensureReady()
        val originWorld = origin.world ?: throw IllegalArgumentException("Origin must have an associated world")
        val destinationWorld = destination.world ?: throw IllegalArgumentException("Destination must have an associated world")
        require(originWorld == destinationWorld) { "Origin and destination must be in the same world" }
        require(::pathWorld.isInitialized && originWorld == pathWorld) { "Requested world is not prepared for path finding" }

        val path = engine.routeWithSnap({ x, y, z -> snap.nearestNode(x, y, z) },
            origin.blockX, origin.blockY, origin.blockZ,
            destination.blockX, destination.blockY, destination.blockZ
        )

        return path.map { node ->
            Location(originWorld, node[0] + 0.5, node[1] + 0.1, node[2] + 0.5)
        }
    }

    val buildCommand: LiteralCommandNode<CommandSourceStack> = LiteralArgumentBuilder.literal<CommandSourceStack>("apf")
        .then(Commands.literal("path")
            .then(Commands.argument("destination's X", DoubleArgumentType.doubleArg())
                .then(Commands.argument("destination's Y", DoubleArgumentType.doubleArg())
                    .then(Commands.argument("destination's Z", DoubleArgumentType.doubleArg())
                        .executes { ctx ->
                            val sender = ctx.source.sender
                            if (sender !is Player) {
                                sender.sendMessage("이 명령어는 플레이어만 사용할 수 있습니다.")
                                return@executes Command.SINGLE_SUCCESS
                            }

                            val tx = DoubleArgumentType.getDouble(ctx, "destination's X").toInt()
                            val ty = DoubleArgumentType.getDouble(ctx, "destination's Y").toInt()
                            val tz = DoubleArgumentType.getDouble(ctx, "destination's Z").toInt()

                            val origin = sender.location.clone()
                            val originWorld = origin.world ?: run {
                                sender.sendMessage("경로를 찾을 수 없습니다 (월드 정보 없음).")
                                return@executes Command.SINGLE_SUCCESS
                            }

                            sender.sendMessage("경로 파티클을 생성합니다...")

                            val destination = Location(originWorld, tx.toDouble(), ty.toDouble(), tz.toDouble())
                            val path = try {
                                api.getPath(origin, destination)
                            } catch (ex: IllegalStateException) {
                                sender.sendMessage("경로를 계산할 수 없습니다: ${ex.message}")
                                return@executes Command.SINGLE_SUCCESS
                            } catch (ex: IllegalArgumentException) {
                                sender.sendMessage("경로를 계산할 수 없습니다: ${ex.message}")
                                return@executes Command.SINGLE_SUCCESS
                            }

                            if (path.isEmpty()) {
                                sender.sendMessage("경로를 찾을 수 없습니다.")
                                return@executes Command.SINGLE_SUCCESS
                            }

                            activePathTasks.remove(sender.uniqueId)?.cancel()

                            val particleLocations = path

                            val task = server.scheduler.runTaskTimer(this, Runnable {
                                if (!sender.isOnline) {
                                    activePathTasks.remove(sender.uniqueId)?.cancel()
                                    return@Runnable
                                }

                                val playerLocation = sender.location

                                particleLocations.forEach { location ->
                                    if (location.world == playerLocation.world &&
                                        location.distanceSquared(playerLocation) <= 9.0
                                    ) {
                                        sender.spawnParticle(Particle.HAPPY_VILLAGER, location, 1, 0.0, 0.0, 0.0, 0.0)
                                    }
                                }
                            }, 0L, 5L)

                            server.scheduler.runTaskLater(this, Runnable {
                                task.cancel()
                                activePathTasks.remove(sender.uniqueId)
                            }, 20L * 30)

                            activePathTasks[sender.uniqueId] = task

                            return@executes Command.SINGLE_SUCCESS
                        }
                    )
                )
            )
        )
        .then(Commands.literal("find")
            .then(Commands.argument("destination's X", DoubleArgumentType.doubleArg())
                .then(Commands.argument("destination's Y", DoubleArgumentType.doubleArg())
                    .then(Commands.argument("destination's Z", DoubleArgumentType.doubleArg())
                        .executes { ctx ->
                            val tx = DoubleArgumentType.getDouble(ctx, "destination's X")
                            val ty = DoubleArgumentType.getDouble(ctx, "destination's Y")
                            val tz = DoubleArgumentType.getDouble(ctx, "destination's Z")
                            val sender = ctx.source.sender as Player
                            val w = sender.world

                            sender.sendMessage("경로 탐색을 시작합니다...")

                            val origin = sender.location.clone()
                            if (origin.world != w) {
                                sender.sendMessage("경로를 찾을 수 없습니다 (다른 월드입니다).")
                                return@executes Command.SINGLE_SUCCESS
                            }

                            val destination = Location(w, tx, ty, tz)
                            val path = try {
                                api.getPath(origin, destination)
                            } catch (ex: IllegalStateException) {
                                sender.sendMessage("경로를 계산할 수 없습니다: ${ex.message}")
                                return@executes Command.SINGLE_SUCCESS
                            } catch (ex: IllegalArgumentException) {
                                sender.sendMessage("경로를 계산할 수 없습니다: ${ex.message}")
                                return@executes Command.SINGLE_SUCCESS
                            }

                            if (path.isEmpty()) {
                                sender.sendMessage("경로를 찾을 수 없습니다.")
                                return@executes Command.SINGLE_SUCCESS
                            }
                            val coords = path.map { p -> "(${p.blockX}, ${p.blockY}, ${p.blockZ})" }

                            // 전체 좌표를 출력 (목적지까지 전부)
                            coords.chunked(6).forEach { chunk ->
                                sender.sendMessage("§a" + chunk.joinToString(" §7→§a "))
                            }

                            return@executes Command.SINGLE_SUCCESS
                        }
                    )
                )
            )
        )
        .then(Commands.literal("debug")
            .then(Commands.argument("x", IntegerArgumentType.integer())
                .then(Commands.argument("y", IntegerArgumentType.integer())
                    .then(Commands.argument("z", IntegerArgumentType.integer())
                        .executes { ctx ->
                            debugNeighborsAt(ctx.source.sender, engine, worldGraph, IntegerArgumentType.getInteger(ctx, "x"), IntegerArgumentType.getInteger(ctx, "y"), IntegerArgumentType.getInteger(ctx, "z"))
                            return@executes Command.SINGLE_SUCCESS
                        }
                    )
                )
            )
        )
        .then(Commands.literal("dumpNodes")
            .executes { ctx ->
                val sender = ctx.source.sender
                if (!::snap.isInitialized) {
                    sender.sendMessage("스냅 인덱스가 아직 초기화되지 않았습니다.")
                    return@executes Command.SINGLE_SUCCESS
                }

                val lines = snap.dumpAllNodes { _ -> }
                
                lines.forEach { line -> println(line) }
                
                return@executes Command.SINGLE_SUCCESS
            }
        )
        .build()
}

fun debugNeighborsAt(sender: CommandSender, engine: QueryEngine, g: NavWorldGraph, x:Int,y:Int,z:Int) {
    val s = engine.javaClass.getDeclaredMethod("nearestNode", Int::class.java, Int::class.java, Int::class.java)
        .apply { isAccessible = true }
        .invoke(engine, x, y, z) as Int
    if (s < 0) { sender.sendMessage("No snap"); return }
    val off = engine.javaClass.getDeclaredField("idx").apply { isAccessible = true }
        .get(engine) as CCHIndex
    val sOff = off.upOff[s]; val e = off.upOff[s + 1]
    var flat=0; var up=0; var down=0
    for (i in sOff until e) {
        val v = off.upTo[i]
        val dy = g.nodeY(v) - g.nodeY(s)
        when {
            dy == 0 -> flat++
            dy > 0  -> up++
            else    -> down++
        }
    }
    sender.sendMessage("neighbors at ($x,$y,$z) => flat=$flat, up=$up, down=$down")
}

fun debugNeighborsAround(
    g: NavWorldGraph,
    x:Int, y:Int, z:Int,
    maxPrint:Int = 32
) {
    // 좌표→노드 스냅(간단 스캔; 느려도 디버그용 OK)
    var s = -1; var best = Long.MAX_VALUE
    val a = g.xyzt
    for (u in 0 until g.nodeCount) {
        val dx = (a[u*3]-x).toLong(); val dy = (a[u*3+1]-y).toLong(); val dz = (a[u*3+2]-z).toLong()
        val d2 = dx*dx + dy*dy + dz*dz
        if (d2 < best) { best = d2; s = u }
    }
    if (s < 0) { println("no snap"); return }

    val sx = a[s*3]; val sy = a[s*3+1]; val sz = a[s*3+2]
    println("node s=$s at ($sx,$sy,$sz)")

    val off = g.csrOff; val to = g.csrTo
    var flat=0; var up=0; var down=0; var cnt=0
    var i = off[s]; val e = off[s+1]
    while (i < e && cnt < maxPrint) {
        val v = to[i]; val vx = a[v*3]; val vy = a[v*3+1]; val vz = a[v*3+2]
        val dy = vy - sy
        val flag = when { dy > 0 -> { up++;   "↑" }
            dy < 0 -> { down++; "↓" }
            else   -> { flat++; "→" } }
        println("  $flag v=$v at ($vx,$vy,$vz)  Δy=$dy")
        i++; cnt++
    }
    println("counts: flat=$flat up=$up down=$down (printed=$cnt)")
}

fun debugUpwardNeighbors(
    g: NavWorldGraph,
    idx: CCHIndex,
    x:Int, y:Int, z:Int,
    maxPrint:Int = 32
) {
    // s 찾기
    var s = -1; var best = Long.MAX_VALUE
    val a = g.xyzt
    for (u in 0 until g.nodeCount) {
        val dx = (a[u*3]-x).toLong(); val dy = (a[u*3+1]-y).toLong(); val dz = (a[u*3+2]-z).toLong()
        val d2 = dx*dx + dy*dy + dz*dz
        if (d2 < best) { best = d2; s = u }
    }
    if (s < 0) { println("no snap"); return }

    val sx = a[s*3]; val sy = a[s*3+1]; val sz = a[s*3+2]
    println("UP-graph at s=$s ($sx,$sy,$sz), rank[s]=${idx.rank[s]}")

    val off = idx.upOff; val to = idx.upTo
    var i = off[s]; val e = off[s+1]
    var k = 0
    while (i < e && k < maxPrint) {
        val v = to[i]; val vx = a[v*3]; val vy = a[v*3+1]; val vz = a[v*3+2]
        println("  → v=$v rank=${idx.rank[v]} at ($vx,$vy,$vz)  Δy=${vy - sy}  mid=${idx.upMid[i]}  w=${idx.upW[i]}")
        i++; k++
    }
}