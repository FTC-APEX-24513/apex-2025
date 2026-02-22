package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.ifHuh
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem
import kotlin.math.abs
import kotlin.math.sign

/**
 * Commands for the Limelight tx-based turret aiming system.
 *
 * Aiming strategy
 * ---------------
 * Tracking relies entirely on Limelight tx — the angular error from the camera
 * crosshair to the goal AprilTag in degrees.  Because the camera sits on the turret,
 * tx is the direct turret error with no coordinate transforms required.
 *
 * Power curve (in TurretSubsystem)
 * ---------------------------------
 * Output power is linearly interpolated between MIN_POWER (at TX_TOLERANCE) and
 * MAX_POWER (at FULL_POWER_TX), so the turret always overcomes static friction even
 * for small errors and reaches MAX_POWER well before the full range of tx.
 *
 * Wire-wrap protection
 * --------------------
 * [TurretSubsystem] enforces LIMIT_MIN/LIMIT_MAX for all automated states.  When
 * tracking, if the direct direction would press against a limit the error is negated
 * here so the turret goes around the other arc.  The Searching branch auto-reverses
 * direction inside the subsystem when a limit is hit.
 *
 * Searching
 * ---------
 * When tx is null, [lockOnGoal] sends a Search message in the last known direction.
 * The subsystem sweeps at SEARCH_POWER and auto-reverses at limits.
 *
 * Alliance selection
 * ------------------
 * Set [RED_GOAL_TAG_ID] / [BLUE_GOAL_TAG_ID] to your goal tag fiducial IDs.
 *
 * Typical usage
 * -------------
 * ```kotlin
 * schedule(loop({ inLoop }, TurretCommands.lockOnGoal(limelight, turret, Alliance.BLUE) { result ->
 *     telemetry.addData("tx",     result.tx)
 *     telemetry.addData("locked", result.isLocked)
 *     telemetry.addData("source", result.source)
 * }))
 * ```
 */
@Config
object TurretCommands {

    // -------------------------------------------------------------------------
    // Alliance / tag-ID configuration
    // -------------------------------------------------------------------------

    @JvmField var RED_GOAL_TAG_ID  = 24
    @JvmField var BLUE_GOAL_TAG_ID = 20

    enum class Alliance {
        RED, BLUE;

        val tagId: Int get() = when (this) {
            RED  -> RED_GOAL_TAG_ID
            BLUE -> BLUE_GOAL_TAG_ID
        }
    }

    enum class TrackingSource { TX, SEARCHING }

    // -------------------------------------------------------------------------
    // Result type
    // -------------------------------------------------------------------------

    data class TrackingResult(
        val alliance: Alliance,
        val tagId: Int,
        val tagVisible: Boolean,
        /** Raw tx from the Limelight for the goal tag, degrees. Null if tag not visible. */
        val tx: Double?,
        val isLocked: Boolean,
        val source: TrackingSource,
        val estimatedPosition: Double,
        /** Limelight result staleness in ms this tick. High values mean the camera is slow. */
        val stalenessMs: Long
    )

    // -------------------------------------------------------------------------
    // Commands
    // -------------------------------------------------------------------------

    /**
     * Builds a tracking Closure that must be placed inside a [loop] by the caller.
     *
     * Each tick:
     *   1. Read tx + result staleness for the goal tag in one snapshot.
     *   2. If the result is stale (> MAX_TRACKING_STALENESS_MS): send Hold so the
     *      turret stops rather than running on outdated data.
     *   3. If tag visible and fresh: apply wire-wrap check, send Track.
     *   4. If tag not visible and fresh: send Search in the last known direction.
     *   5. Fire [onResult] with a diagnostic snapshot.
     */
    fun lockOnGoal(
        limelight: LimelightSubsystem,
        turret: TurretSubsystem,
        alliance: Alliance,
        onResult: ((TrackingResult) -> Unit)? = null
    ): Closure {
        val tagId = alliance.tagId

        // Plain vars — captured by closure, safe across fiber boundaries.
        var effectiveTx: Double  = 0.0   // tx after wire-wrap flip, read by turret.track lambda
        var tagVisible: Boolean  = false
        var stale: Boolean       = false
        var lastKnownDir: Double = 1.0

        return sequence(
            exec {
                val snapshot = limelight.getTxAndStaleness(tagId)
                stale = snapshot == null || snapshot.second > TurretSubsystem.MAX_TRACKING_STALENESS_MS
                val currentTx = snapshot?.first

                tagVisible = !stale && currentTx != null

                if (tagVisible && currentTx != null) {
                    if (currentTx != 0.0) lastKnownDir = sign(currentTx)

                    // Wire-wrap check: if tracking in the current direction would hit a limit,
                    // negate the error so the turret goes around the other arc instead.
                    val atCWLimit  = turret.estimatedPosition >= TurretSubsystem.LIMIT_MAX && currentTx > 0.0
                    val atCCWLimit = turret.estimatedPosition <= TurretSubsystem.LIMIT_MIN && currentTx < 0.0
                    effectiveTx = if (atCWLimit || atCCWLimit) -currentTx else currentTx
                }

                onResult?.invoke(
                    TrackingResult(
                        alliance          = alliance,
                        tagId             = tagId,
                        tagVisible        = tagVisible,
                        tx                = currentTx,
                        isLocked          = tagVisible && abs(effectiveTx) <= TurretSubsystem.TX_TOLERANCE,
                        source            = when {
                            stale      -> TrackingSource.SEARCHING
                            tagVisible -> TrackingSource.TX
                            else       -> TrackingSource.SEARCHING
                        },
                        estimatedPosition = turret.estimatedPosition,
                        stalenessMs       = snapshot?.second ?: -1L
                    )
                )
            },
            ifHuh(
                { stale },
                // Stale result — hold in place rather than acting on bad data.
                turret.hold()
            ).elseHuh(
                ifHuh(
                    { tagVisible },
                    turret.track { effectiveTx }
                ).elseHuh(
                    turret.search { lastKnownDir }
                )
            )
        )
    }

    /**
     * One-shot aim — send one Track or Search command and complete immediately.
     * For continuous locking use [lockOnGoal] inside a loop.
     */
    fun aimOnce(
        limelight: LimelightSubsystem,
        turret: TurretSubsystem,
        alliance: Alliance
    ): Closure {
        val tx = limelight.getTargetTxForTag(alliance.tagId)
        return if (tx != null) turret.track(tx) else turret.search()
    }

    /**
     * Cut turret power immediately.
     */
    fun stopTurret(turret: TurretSubsystem): Closure = turret.hold()
}
