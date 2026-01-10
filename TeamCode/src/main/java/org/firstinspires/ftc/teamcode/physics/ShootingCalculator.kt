package org.firstinspires.ftc.teamcode.physics

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import kotlin.math.*

/**
 * Physics engine for calculating shooting trajectories and flywheel speeds.
 * Uses projectile motion equations to determine optimal shot parameters.
 */
object ShootingCalculator {
    private const val GRAVITY = 9.81  // m/s²
    
    /**
     * Calculate required ball velocity to hit GOAL at given distance.
     * Uses simplified projectile motion for horizontal launch.
     * 
     * @param horizontalDistance Distance to GOAL in meters
     * @param launchAngle Launch angle in radians (0 = horizontal)
     * @return Required velocity in m/s, or null if shot is impossible
     */
    fun calculateRequiredVelocity(
        horizontalDistance: Double,
        launchAngle: Double = OuttakeSubsystem.LAUNCH_ANGLE_RADIANS
    ): Double? {
        val heightDiff = LimelightSubsystem.GOAL_TOP_LIP_HEIGHT_METERS -
                        OuttakeSubsystem.LAUNCH_HEIGHT_METERS

        // Physics: Projectile Motion for Velocity
        // y = x * tan(theta) - (g * x^2) / (2 * v^2 * cos^2(theta))
        // Solve for v:
        // v = sqrt( (g * x^2) / (2 * cos^2(theta) * (x * tan(theta) - y)) )

        val x = horizontalDistance
        val y = heightDiff
        val theta = launchAngle
        val g = GRAVITY

        val cosTheta = cos(theta)
        val tanTheta = tan(theta)

        val numerator = g * x * x
        val denominator = 2 * cosTheta * cosTheta * (x * tanTheta - y)

        if (denominator <= 0) return null // Shot impossible (target too high for this angle/distance)

        return sqrt(numerator / denominator)
    }
    
    /**
     * Convert required ball velocity to flywheel RPM.
     * 
     * @param ballVelocity Required ball velocity in m/s
     * @return Required flywheel RPM
     */
    fun velocityToRPM(ballVelocity: Double): Double {
        // Ball velocity = flywheel surface velocity * efficiency
        // v_ball = (ω * r) * efficiency
        // ω (rad/s) = v_ball / (r * efficiency)
        // RPM = ω * 60 / (2π)
        
        val angularVelocity = ballVelocity / 
            (OuttakeSubsystem.FLYWHEEL_RADIUS_METERS *
             OuttakeSubsystem.FLYWHEEL_TO_BALL_EFFICIENCY)
        
        return angularVelocity * 60.0 / (2.0 * PI)
    }
    
    /**
     * Convert flywheel RPM to expected ball velocity.
     * 
     * @param rpm Flywheel RPM
     * @return Expected ball velocity in m/s
     */
    fun rpmToVelocity(rpm: Double): Double {
        val angularVelocity = rpm * 2.0 * PI / 60.0  // Convert to rad/s
        return angularVelocity * OuttakeSubsystem.FLYWHEEL_RADIUS_METERS *
               OuttakeSubsystem.FLYWHEEL_TO_BALL_EFFICIENCY
    }
    
    /**
     * Calculate complete shot parameters for a given distance.
     * Returns all relevant shooting data or null if shot is not feasible.
     * 
     * @param distance Distance to GOAL in meters
     * @param source String describing distance source ("Limelight" or "Odometry")
     * @return ShotParameters or null if shot impossible
     */
    fun calculateShotParameters(distance: Double): ShotParameters? {
        if (distance < OuttakeSubsystem.MIN_SHOT_DISTANCE_METERS ||
            distance > OuttakeSubsystem.MAX_SHOT_DISTANCE_METERS) {
            return null
        }

        val velocity = calculateRequiredVelocity(distance) ?: return null
        
        // Convert to RPM
        val rpm = velocityToRPM(velocity)
        
        // Check RPM bounds
        if (rpm > OuttakeSubsystem.MAX_FLYWHEEL_RPM ||
            rpm < OuttakeSubsystem.MIN_STABLE_RPM) {
            return null
        }
        
        return ShotParameters(
            targetRPM = rpm,
            distance = distance,
            requiredVelocity = velocity,
        )
    }
    
    /**
     * Calculate 2D Euclidean distance between two points.
     * 
     * @param x1 X coordinate of first point
     * @param y1 Y coordinate of first point
     * @param x2 X coordinate of second point
     * @param y2 Y coordinate of second point
     * @return Distance between points
     */
    fun distance2D(x1: Double, y1: Double, x2: Double, y2: Double): Double {
        val dx = x2 - x1
        val dy = y2 - y1
        return sqrt(dx*dx + dy*dy)
    }
    
    /**
     * Calculate flywheel RPM after a shot using energy conservation.
     * 
     * Physics:
     * - Initial rotational KE: KE_rot = 0.5 * I * ω²
     * - Ball KE: KE_ball = 0.5 * m * v²
     * - Final rotational KE: KE_final = KE_rot - KE_ball
     * - Final RPM: ω_final = sqrt(2 * KE_final / I)
     * 
     * @param currentRPM Flywheel RPM before shot
     * @return Flywheel RPM immediately after ball leaves contact
     */
    fun calculateRPMAfterShot(currentRPM: Double): Double {
        // Convert RPM to angular velocity (rad/s)
        val omega_initial = currentRPM * 2.0 * PI / 60.0
        
        // Initial rotational kinetic energy
        val I = OuttakeSubsystem.FLYWHEEL_MOMENT_OF_INERTIA
        val KE_rotational = 0.5 * I * omega_initial * omega_initial
        
        // Ball velocity from flywheel (accounts for efficiency)
        val v_ball = rpmToVelocity(currentRPM)
        
        // Energy transferred to ball
        val m_ball = OuttakeSubsystem.BALL_MASS_KG
        val KE_ball = 0.5 * m_ball * v_ball * v_ball
        
        // Remaining rotational energy
        val KE_final = (KE_rotational - KE_ball).coerceAtLeast(0.0)
        
        // Final angular velocity
        val omega_final = sqrt(2.0 * KE_final / I)
        
        // Convert back to RPM
        return omega_final * 60.0 / (2.0 * PI)
    }
    
    /**
     * Calculate time for motor to re-accelerate flywheel from post-shot RPM back to target.
     * Uses motor torque curve and flywheel inertia for accurate prediction.
     * 
     * @param targetRPM Desired flywheel RPM
     * @param currentRPM Current flywheel RPM (post-shot)
     * @return Time in seconds to reach target RPM
     */
    fun calculateReaccelerationTime(targetRPM: Double, currentRPM: Double): Double {
        if (targetRPM <= currentRPM) return 0.0
        
        val rpm_delta = targetRPM - currentRPM
        
        // Time = Δrpm / acceleration_rate
        // Acceleration rate from motor torque and flywheel inertia
        val time = rpm_delta / OuttakeSubsystem.MOTOR_ACCELERATION_RATE_RPM_PER_SEC
        
        // Add safety margin (20%) and enforce minimum
        return (time * OuttakeSubsystem.RECOVERY_TIME_SAFETY_FACTOR)
            .coerceAtLeast(OuttakeSubsystem.MIN_RECOVERY_TIME_SECONDS)
    }
    
    /**
     * Calculate complete recovery time after a shot.
     * Combines energy loss calculation with motor re-acceleration.
     * 
     * @param targetRPM The RPM the flywheel should return to
     * @return Recovery time in seconds
     */
    fun calculateRecoveryTime(targetRPM: Double): Double {
        val rpmAfterShot = calculateRPMAfterShot(targetRPM)
        return calculateReaccelerationTime(targetRPM, rpmAfterShot)
    }
    
    /**
     * Calculate ball contact time with flywheel (informational).
     * Contact time = arc_length / surface_velocity
     * 
     * @param rpm Current flywheel RPM
     * @return Contact time in seconds
     */
    fun calculateBallContactTime(rpm: Double): Double {
        val omega = rpm * 2.0 * PI / 60.0  // rad/s
        val contact_angle_rad = OuttakeSubsystem.BALL_CONTACT_ARC_DEGREES * PI / 180.0
        return contact_angle_rad / omega
    }
    
    /**
     * Calculate spinup time from current RPM to target RPM.
     * Used for initial spinup and telemetry.
     * 
     * @param currentRPM Starting RPM
     * @param targetRPM Desired RPM
     * @return Estimated spinup time in seconds
     */
    fun calculateSpinupTime(currentRPM: Double, targetRPM: Double): Double {
        return calculateReaccelerationTime(targetRPM, currentRPM)
    }
}
