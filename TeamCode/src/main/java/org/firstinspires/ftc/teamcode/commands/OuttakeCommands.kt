package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.util.LaunchParametersSolver
import org.firstinspires.ftc.teamcode.util.ShotTable

/**
 * Commands for the outtake subsystem that integrate with the Limelight
 * for automatic aiming.
 *
 * Shot parameter strategy
 * -----------------------
 * Two modes are available and can be toggled live from FTC Dashboard:
 *
 *   USE_PHYSICS_SOLVER = false (default)
 *     Uses [ShotTable] — a manually tuned lookup table of (distance → RPM, hood).
 *     Interpolates linearly between tuned rows.  Fast to tune, robust, recommended.
 *
 *   USE_PHYSICS_SOLVER = true
 *     Uses [LaunchParametersSolver] — a physics model with drag/lift simulation.
 *     Requires careful calibration of drag, lift, and motor constants.
 *     Keep as a reference / for comparison.
 *
 * Distance source
 * ---------------
 * [LimelightSubsystem.getDistanceToGoal] tries in order:
 *   1. MegaTag2 full-field pose (most accurate, needs ≥1 tag visible)
 *   2. ty geometric formula    (needs goal tag visible, no full localisation)
 *   3. Pinpoint odometry       (always available, may drift)
 */
@Config
object OuttakeCommands {

    /** Tag ID of the goal AprilTag for the current alliance. Change for RED. */
    @JvmField var GOAL_TAG_ID = TurretCommands.BLUE_GOAL_TAG_ID

    /**
     * When true, use [LaunchParametersSolver] instead of [ShotTable].
     * Toggle live from FTC Dashboard to A/B compare both approaches.
     */
    @JvmField var USE_PHYSICS_SOLVER = false

    // -------------------------------------------------------------------------
    // Result type
    // -------------------------------------------------------------------------

    /**
     * Result of a shot calculation.
     *
     * @param distanceMeters  Distance to goal in metres.
     * @param distanceSource  Which sensor was used to measure distance.
     * @param rpm             Target flywheel RPM.
     * @param hoodAngle       Target hood angle in degrees.
     * @param success         False if calculation failed (table not tuned, out of range, etc.)
     */
    data class AimResult(
        val distanceMeters: Double,
        val distanceSource: LimelightSubsystem.DistanceSource?,
        val rpm: Double,
        val hoodAngle: Double,
        val success: Boolean
    ) {
        companion object {
            val FAILED = AimResult(0.0, null, 0.0, 0.0, false)
        }
    }

    // -------------------------------------------------------------------------
    // Core calculation
    // -------------------------------------------------------------------------

    /**
     * Calculate the optimal shot parameters for the robot's current position.
     *
     * Uses [ShotTable] by default (or [LaunchParametersSolver] if [USE_PHYSICS_SOLVER]).
     * Distance is obtained via [LimelightSubsystem.getDistanceToGoal] with its
     * three-source fallback chain.
     */
    fun calculateShot(limelight: LimelightSubsystem): AimResult {
        val distResult = limelight.getDistanceToGoal(GOAL_TAG_ID)
        val dist = distResult.distanceMeters

        return if (USE_PHYSICS_SOLVER) {
            val result = LaunchParametersSolver.solve(dist) ?: return AimResult.FAILED
            AimResult(
                distanceMeters = dist,
                distanceSource = distResult.source,
                rpm            = result.rpm,
                hoodAngle      = result.angle.toDouble(),
                success        = true
            )
        } else {
            if (!ShotTable.isTuned) return AimResult.FAILED
            val (rpm, hood) = ShotTable.lookup(dist)
            if (rpm <= 0.0) return AimResult.FAILED
            AimResult(
                distanceMeters = dist,
                distanceSource = distResult.source,
                rpm            = rpm,
                hoodAngle      = hood,
                success        = true
            )
        }
    }

    // -------------------------------------------------------------------------
    // Commands
    // -------------------------------------------------------------------------

    /**
     * Re-aim the outtake every time this closure is executed.
     *
     * Calculates the shot parameters lazily at send time (inside the Channels.send
     * supplier), so placing this inside a [loop] correctly recalculates each tick
     * as the robot moves.  If the calculation fails (table not tuned, no distance
     * source), sends a Stop request instead.
     */
    fun aimOuttake(limelight: LimelightSubsystem, outtake: OuttakeSubsystem): Closure =
        Channels.send(
            {
                val aim = calculateShot(limelight)
                if (aim.success) OuttakeSubsystem.Request.SetTarget(aim.rpm, aim.hoodAngle)
                else             OuttakeSubsystem.Request.Stop
            },
            { outtake.actor.tx }
        )

    /**
     * Aim the outtake (one calculation at call time) and then block until the
     * flywheel reaches the target RPM.  Use for a fire-when-ready sequence;
     * not for continuous tracking.
     */
    fun aimAndWaitForReady(limelight: LimelightSubsystem, outtake: OuttakeSubsystem): Closure {
        val aim = calculateShot(limelight)
        return sequence(
            if (aim.success) outtake.spinUp(aim.rpm, aim.hoodAngle) else outtake.stop(),
            wait { outtake.isAtTargetRPM() }
        )
    }
}
