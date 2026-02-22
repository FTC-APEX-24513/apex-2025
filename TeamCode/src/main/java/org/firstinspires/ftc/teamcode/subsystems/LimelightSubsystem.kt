package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.di.HardwareFactory
import org.firstinspires.ftc.teamcode.di.HardwareScope
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.tan

@Config
@Inject
@HardwareScope
class LimelightSubsystem(factory: HardwareFactory) : Subsystem<LimelightSubsystem.Pipeline, LimelightSubsystem.Pipeline>() {

    private val limelight = factory.getLimelight("limelight").also {
        it.setPollRateHz(90)
        it.start()
    }

    private val pinpoint = factory.getPinpoint("pinpoint")

    companion object {
        const val METERS_PER_INCH = 0.0254
        const val INCHES_PER_METER = 1.0 / METERS_PER_INCH

        // Goal position on the field (inches, FTC standard coordinate system).
        @JvmField var BLUE_GOAL_X_INCHES = 16.0
        @JvmField var BLUE_GOAL_Y_INCHES = 132.0
        @JvmField var RED_GOAL_X_INCHES  = 129.0
        @JvmField var RED_GOAL_Y_INCHES  = 132.0

        // Camera mounting geometry — measure these on the physical robot and tune here.
        /** Height of the Limelight lens above the floor, in metres. */
        @JvmField var CAMERA_HEIGHT_M = 0.30
        /** Upward tilt of the Limelight from horizontal, in degrees. */
        @JvmField var CAMERA_MOUNT_ANGLE_DEG = 20.0
        /** Height of the centre of the goal AprilTag above the floor, in metres. */
        @JvmField var GOAL_TAG_HEIGHT_M = 0.60

        // Limelight physical offset from the robot centre, in inches.
        // Measured along the robot's X axis (forward) and Y axis (left).
        // The MT2 bot-pose is reported at the camera lens; these offsets correct it back
        // to the robot centre so that the pinpoint can be reset accurately.
        // Limelight lens offset from robot centre (forward = +X, left = +Y), in inches.
        @JvmField var LL_OFFSET_X_INCHES = 6.372480315
        @JvmField var LL_OFFSET_Y_INCHES = 12.165038189

        // Turret pivot offset from robot centre (forward = +X, left = +Y), in inches.
        @JvmField var SHOOTER_CENTER_OFFSET_X_INCHES = 0.0
        @JvmField var SHOOTER_CENTER_OFFSET_Y_INCHES = 9.5344488189

        // Sanity-check: if MT2 pose disagrees with current odo by more than this (inches),
        // the result is considered frozen/stale and ignored.
        @JvmField var RELOCALIZE_MAX_JUMP_INCHES = 20.0

        // Maximum staleness (ms) before a Limelight result is discarded.
        @JvmField var MAX_STALENESS_MS = 50L
    }

    /** 2-D pose of the robot centre in inches (FTC field coordinate system). */
    data class RobotPose(val x: Double, val y: Double, val headingRad: Double)

    enum class Pipeline {
        APRILTAG
    }

    override val initialState = { Pipeline.APRILTAG }

    override val transition = { _: Pipeline, newPipeline: Pipeline ->
        limelight.pipelineSwitch(newPipeline.ordinal)
        newPipeline
    }

    override val behavior = { register: VarRegister<Pipeline> ->
        match { register.get() }
            .branch(Pipeline.APRILTAG, loop(exec {
                pinpoint.update()
                val heading = pinpoint.getHeading(AngleUnit.DEGREES)
                limelight.updateRobotOrientation(heading)
            }))
            .assertExhaustive()
    }

    fun useAprilTagPipeline(): Closure = update(Pipeline.APRILTAG)

    // -------------------------------------------------------------------------
    // Odometry — direct pinpoint reads (pinpoint.update() runs in behavior loop)
    // -------------------------------------------------------------------------

    /** Current robot X position in inches from pinpoint odometry. */
    fun getRobotX(): Double = pinpoint.getPosX(DistanceUnit.INCH)

    /** Current robot Y position in inches from pinpoint odometry. */
    fun getRobotY(): Double = pinpoint.getPosY(DistanceUnit.INCH)

    /** Current robot heading in radians from pinpoint odometry. */
    fun getRobotHeadingRad(): Double = pinpoint.getHeading(AngleUnit.RADIANS)

    /** Current robot heading in degrees from pinpoint odometry. */
    fun getRobotHeadingDeg(): Double = pinpoint.getHeading(AngleUnit.DEGREES)

    /** Current robot pose (x/y inches, heading radians) from pinpoint odometry. */
    fun getRobotPose(): RobotPose = RobotPose(getRobotX(), getRobotY(), getRobotHeadingRad())

    // -------------------------------------------------------------------------
    // MegaTag2 relocalization
    // -------------------------------------------------------------------------

    /**
     * Attempt to relocalize the robot using the Limelight's MegaTag2 bot-pose.
     *
     * The MT2 pose is reported at the camera lens centre.  This function:
     *   1. Rotates the camera→robot-centre offset vector by the current robot heading.
     *   2. Subtracts it from the MT2 position to get the robot centre in field coordinates.
     *   3. Sanity-checks: staleness < [MAX_STALENESS_MS] and displacement < [RELOCALIZE_MAX_JUMP_INCHES].
     *   4. On success, resets the pinpoint to the corrected pose so all subsequent
     *      odometry reads are anchored to the field coordinate system.
     *
     * @param turretAngleRad Current turret angle relative to robot heading, in radians.
     *                       Used to rotate the shooter offset into field coordinates.
     * @return The corrected robot-centre pose in inches if relocalization succeeded, null otherwise.
     */
    fun relocalize(turretAngleRad: Double = 0.0): RobotPose? {
        val result = limelight.latestResult ?: return null
        if (!result.isValid || result.getStaleness() > MAX_STALENESS_MS) return null

        val mt2 = result.botpose_MT2 ?: return null

        // MT2 pose is in metres, WCS.  The WCS is rotated 90° relative to the FTC coordinate
        // system, so we rotate by -PI/2 to align axes.
        val rawX = mt2.position.x  // metres
        val rawY = mt2.position.y  // metres
        val correctedXM = rawX * cos(-Math.PI / 2) - rawY * sin(-Math.PI / 2)
        val correctedYM = rawX * sin(-Math.PI / 2) + rawY * cos(-Math.PI / 2)
        val correctedXIn = correctedXM * INCHES_PER_METER
        val correctedYIn = correctedYM * INCHES_PER_METER

        // Current robot heading (from pinpoint, which was just updated this tick).
        val heading = getRobotHeadingRad()

        // LL offset rotated into field frame — corrects MT2 position from lens to robot centre.
        val llOffX = LL_OFFSET_X_INCHES * cos(heading) - LL_OFFSET_Y_INCHES * sin(heading)
        val llOffY = LL_OFFSET_X_INCHES * sin(heading) + LL_OFFSET_Y_INCHES * cos(heading)

        val robotCentreX = correctedXIn - llOffX
        val robotCentreY = correctedYIn - llOffY

        // Sanity check: reject if the jump from current odo is implausibly large.
        val currentX = getRobotX()
        val currentY = getRobotY()
        if (hypot(robotCentreX - currentX, robotCentreY - currentY) > RELOCALIZE_MAX_JUMP_INCHES) {
            return null
        }

        // Reset pinpoint to the corrected pose, preserving the current heading.
        pinpoint.setPosition(
            Pose2D(DistanceUnit.INCH, robotCentreX, robotCentreY, AngleUnit.RADIANS, heading)
        )

        return RobotPose(robotCentreX, robotCentreY, heading)
    }

    // -------------------------------------------------------------------------
    // Tag reads
    // -------------------------------------------------------------------------

    fun getBotPoseMT2(): Pose3D? {
        val result = limelight.latestResult
        if (result == null || !result.isValid) return null
        return result.botpose_MT2
    }

    /**
     * Get the horizontal offset to the target (tx).
     * Negative = target is left of crosshair, Positive = right.
     * @return tx value in degrees, or null if no valid target
     */
    fun getTargetTx(): Double? {
        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null
        return result.tx
    }

    /**
     * Get the vertical offset to the target (ty).
     * Negative = target is below crosshair, Positive = above.
     * @return ty value in degrees, or null if no valid target
     */
    fun getTargetTy(): Double? {
        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null
        return result.ty
    }

    /**
     * Check if the Limelight currently has a valid target.
     */
    fun hasTarget(): Boolean {
        val result = limelight.latestResult ?: return false
        return result.isValid
    }

    /**
     * Get the horizontal offset (tx) to a specific AprilTag by its fiducial ID.
     * Searches the full fiducial result list so the desired tag doesn't have to be
     * the primary target (highest-confidence detection).
     *
     * @param tagId The AprilTag fiducial ID to search for.
     * @return Horizontal offset in degrees (negative = tag is left, positive = right),
     *         or null if the tag is not currently visible.
     */
    fun getTargetTxForTag(tagId: Int): Double? {
        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null
        return result.fiducialResults
            .firstOrNull { it.fiducialId == tagId }
            ?.targetXDegrees
    }

    /**
     * Returns tx for [tagId] together with the result staleness in milliseconds.
     * Staleness is how long ago the Limelight produced this frame.
     * Both are read from the same latestResult snapshot so they are consistent.
     *
     * @return Pair(tx, stalenessMs), or null if no valid result exists.
     *         tx is null if the tag is not in this frame.
     */
    fun getTxAndStaleness(tagId: Int): Pair<Double?, Long>? {
        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null
        val tx = result.fiducialResults
            .firstOrNull { it.fiducialId == tagId }
            ?.targetXDegrees
        return Pair(tx, result.getStaleness())
    }

    /**
     * Check whether a specific AprilTag is currently visible.
     *
     * @param tagId The AprilTag fiducial ID to search for.
     */
    fun hasTag(tagId: Int): Boolean = getTargetTxForTag(tagId) != null

    // -------------------------------------------------------------------------
    // Distance to goal — three independent sources for cross-validation.
    // -------------------------------------------------------------------------

    /**
     * Distance via MegaTag2 full-field pose estimation.
     * Most accurate when multiple AprilTags are visible.
     * Returns null if no valid pose is available.
     */
    fun getDistanceMT2(): Double? {
        val pose = getBotPoseMT2() ?: return null
        val goalX = BLUE_GOAL_X_INCHES * METERS_PER_INCH
        val goalY = BLUE_GOAL_Y_INCHES * METERS_PER_INCH
        return hypot(goalX - pose.position.x, goalY - pose.position.y)
    }

    /**
     * Distance via ty geometric formula using the goal AprilTag's vertical offset.
     * Works as long as the tag is visible; does not require full-field localisation.
     *
     * Formula: dist = (GOAL_TAG_HEIGHT_M - CAMERA_HEIGHT_M) /
     *                  tan(toRadians(CAMERA_MOUNT_ANGLE_DEG + ty))
     *
     * Tune CAMERA_HEIGHT_M, CAMERA_MOUNT_ANGLE_DEG, and GOAL_TAG_HEIGHT_M in
     * FTC Dashboard until this reading matches a tape-measure distance.
     *
     * @param tagId AprilTag fiducial ID of the goal tag to use.
     * Returns null if the tag is not currently visible.
     */
    fun getDistanceTy(tagId: Int): Double? {
        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null
        val tag = result.fiducialResults.firstOrNull { it.fiducialId == tagId } ?: return null
        val ty = tag.targetYDegrees
        val angle = CAMERA_MOUNT_ANGLE_DEG + ty
        if (angle <= 0.0) return null          // tag below horizon — degenerate
        return (GOAL_TAG_HEIGHT_M - CAMERA_HEIGHT_M) / tan(Math.toRadians(angle))
    }

    /**
     * Distance via Pinpoint odometry.
     * Always available (no camera required), but depends on the goal coordinates
     * being set correctly and the odometry not having drifted significantly.
     */
    fun getDistancePinpoint(): Double {
        val robotX = pinpoint.getPosX(DistanceUnit.INCH)
        val robotY = pinpoint.getPosY(DistanceUnit.INCH)
        return hypot(BLUE_GOAL_X_INCHES - robotX, BLUE_GOAL_Y_INCHES - robotY)
    }

    /**
     * Best available distance to goal, using the fallback chain:
     *   1. MegaTag2 (most accurate, needs tags)
     *   2. ty geometric (needs goal tag visible)
     *   3. Pinpoint odometry (always available)
     *
     * @param goalTagId AprilTag fiducial ID of the goal tag (for ty fallback).
     * @return Distance in metres and the source used as a [DistanceResult].
     */
    fun getDistanceToGoal(goalTagId: Int): DistanceResult {
        getDistanceMT2()?.let { return DistanceResult(it, DistanceSource.MEGATAG2) }
        getDistanceTy(goalTagId)?.let { return DistanceResult(it, DistanceSource.TY_GEOMETRIC) }
        return DistanceResult(getDistancePinpoint() * METERS_PER_INCH, DistanceSource.PINPOINT)
    }

    enum class DistanceSource { MEGATAG2, TY_GEOMETRIC, PINPOINT }

    data class DistanceResult(val distanceMeters: Double, val source: DistanceSource)
}
