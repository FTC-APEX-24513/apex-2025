package org.firstinspires.ftc.teamcode.commands

import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.util.LaunchParametersSolver
import kotlin.math.hypot

/**
 * Commands for the outtake subsystem that integrate with the Limelight
 * and ShooterUltimate solver for automatic aiming.
 */
object OuttakeCommands {

    /**
     * Result of a shot calculation.
     */
    data class AimResult(
        val distance: Double,
        val angle: Int,
        val rpm: Double,
        val velocity: Double,
        val success: Boolean
    ) {
        companion object {
            val FAILED = AimResult(0.0, 0, 0.0, 0.0, false)
        }
    }

    /**
     * Calculate the distance to the goal from the robot's current position.
     * Uses the Limelight's MegaTag2 pose estimation.
     * 
     * @param limelight The limelight subsystem to get pose from
     * @return Distance to goal in meters, or null if pose unavailable
     */
    fun getDistanceToGoal(limelight: LimelightSubsystem): Double? {
        val pose = limelight.getBotPoseMT2() ?: return null
        
        // Get robot position in meters
        val robotX = pose.position.x
        val robotY = pose.position.y
        
        // Goal position in meters (convert from inches)
        val goalX = LimelightSubsystem.BLUE_GOAL_X_INCHES * LimelightSubsystem.METERS_PER_INCH
        val goalY = LimelightSubsystem.BLUE_GOAL_Y_INCHES * LimelightSubsystem.METERS_PER_INCH
        
        // Calculate distance
        return hypot(goalX - robotX, goalY - robotY)
    }

    /**
     * Calculate the optimal shot parameters for the current distance to goal.
     * 
     * @param limelight The limelight subsystem to get pose from
     * @return AimResult with the calculated parameters, or FAILED if calculation fails
     */
    fun calculateShot(limelight: LimelightSubsystem): AimResult {
        val distance = getDistanceToGoal(limelight) ?: return AimResult.FAILED
        
        val result = LaunchParametersSolver.solve(distance) ?: return AimResult.FAILED
        
        return AimResult(
            distance = distance,
            angle = result.angle,
            rpm = result.rpm,
            velocity = result.velocity,
            success = true
        )
    }

    /**
     * Calculate shot parameters for a given distance (useful for testing).
     * 
     * @param distanceMeters Distance to goal in meters
     * @return AimResult with the calculated parameters, or FAILED if calculation fails
     */
    fun calculateShotForDistance(distanceMeters: Double): AimResult {
        val result = LaunchParametersSolver.solve(distanceMeters) ?: return AimResult.FAILED
        
        return AimResult(
            distance = distanceMeters,
            angle = result.angle,
            rpm = result.rpm,
            velocity = result.velocity,
            success = true
        )
    }

    /**
     * Command to aim the outtake based on Limelight data.
     * Calculates the optimal RPM and hood angle using ShooterUltimate,
     * then commands the outtake to spin up accordingly.
     * 
     * This is a one-shot command - it calculates once and sets the target.
     * For continuous tracking, call this command repeatedly.
     * 
     * @param limelight The limelight subsystem for pose estimation
     * @param outtake The outtake subsystem to control
     * @return Closure that aims the outtake
     */
    fun aimOuttake(limelight: LimelightSubsystem, outtake: OuttakeSubsystem): Closure {
        return exec {
            val aimResult = calculateShot(limelight)
            if (aimResult.success) {
                // Use sequence to actually schedule the spinUp command
                outtake.spinUp(aimResult.rpm, aimResult.angle.toDouble())
            }
        }
    }

    /**
     * Command to aim the outtake for a specific distance.
     * Useful for testing or when you have a known distance.
     * 
     * @param distanceMeters Distance to goal in meters
     * @param outtake The outtake subsystem to control
     * @return Closure that aims the outtake, or stops it if no solution
     */
    fun aimOuttakeForDistance(distanceMeters: Double, outtake: OuttakeSubsystem): Closure {
        return exec {
            val aimResult = calculateShotForDistance(distanceMeters)
            if (aimResult.success) {
                outtake.spinUp(aimResult.rpm, aimResult.angle.toDouble())
            } else {
                outtake.stop()
            }
        }
    }

    /**
     * Command sequence to aim and wait until the outtake is at target RPM.
     * 
     * @param limelight The limelight subsystem for pose estimation
     * @param outtake The outtake subsystem to control
     * @return Closure that aims and waits for ready state
     */
    fun aimAndWaitForReady(limelight: LimelightSubsystem, outtake: OuttakeSubsystem): Closure {
        return sequence(
            aimOuttake(limelight, outtake),
            exec {
                // This will block until at target
                while (!outtake.isAtTargetRPM()) {
                    Thread.yield()
                }
            }
        )
    }
}
