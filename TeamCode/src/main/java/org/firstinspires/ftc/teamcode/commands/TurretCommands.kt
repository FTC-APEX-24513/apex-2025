package org.firstinspires.ftc.teamcode.commands

import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem

/**
 * Commands for the turret subsystem that integrate with the Limelight
 * for automatic target tracking.
 * 
 * The turret subsystem is kept agnostic of the Limelight - this command file
 * bridges the gap by reading from Limelight and commanding the turret.
 * 
 * Usage patterns:
 * 1. One-shot aiming: Call aimTurret() on a button press
 * 2. Continuous tracking: Call trackTarget() in a loop while a button is held
 * 3. Manual control: Directly use turret.setAngle() bypassing these commands
 */
object TurretCommands {

    /**
     * Result of a turret aim calculation.
     */
    data class TurretAimResult(
        val targetTx: Double,        // Raw tx value from Limelight
        val currentAngle: Double,    // Current turret angle
        val targetAngle: Double,     // Calculated target angle
        val isInDeadZone: Boolean,   // Whether target would be in dead zone
        val success: Boolean         // Whether calculation succeeded
    ) {
        companion object {
            val FAILED = TurretAimResult(0.0, 0.0, 0.0, false, false)
        }
    }

    /**
     * Get the horizontal offset to target from Limelight (tx value).
     * 
     * @param limelight The limelight subsystem
     * @return tx value in degrees, or null if no valid target
     */
    fun getTargetOffset(limelight: LimelightSubsystem): Double? {
        return limelight.getTargetTx()
    }

    /**
     * Calculate the absolute turret angle needed to point at the Limelight target.
     * Uses current turret angle + Limelight tx offset.
     * 
     * @param limelight The limelight subsystem for target detection
     * @param turret The turret subsystem for current angle
     * @return TurretAimResult with calculated parameters
     */
    fun calculateTurretAngle(limelight: LimelightSubsystem, turret: TurretSubsystem): TurretAimResult {
        val tx = getTargetOffset(limelight) ?: return TurretAimResult.FAILED
        
        val currentAngle = turret.getCurrentAngle()
        
        // tx is the offset from center:
        // Negative tx = target is left of crosshair (need to rotate counter-clockwise)
        // Positive tx = target is right of crosshair (need to rotate clockwise)
        // 
        // To center the target, we add tx to current angle
        // (assuming positive angles = clockwise rotation when viewed from above)
        var targetAngle = currentAngle + tx
        
        // Normalize to 0-360
        targetAngle = ((targetAngle % 360.0) + 360.0) % 360.0
        
        // Check if target would be in dead zone
        val isInDeadZone = turret.isInDeadZone(targetAngle)
        
        return TurretAimResult(
            targetTx = tx,
            currentAngle = currentAngle,
            targetAngle = targetAngle,
            isInDeadZone = isInDeadZone,
            success = true
        )
    }

    /**
     * Calculate turret angle for a specific offset (useful for testing).
     * 
     * @param offsetDegrees Horizontal offset in degrees (like tx)
     * @param turret The turret subsystem for current angle
     * @return TurretAimResult with calculated parameters
     */
    fun calculateTurretAngleForOffset(offsetDegrees: Double, turret: TurretSubsystem): TurretAimResult {
        val currentAngle = turret.getCurrentAngle()
        
        var targetAngle = currentAngle + offsetDegrees
        targetAngle = ((targetAngle % 360.0) + 360.0) % 360.0
        
        val isInDeadZone = turret.isInDeadZone(targetAngle)
        
        return TurretAimResult(
            targetTx = offsetDegrees,
            currentAngle = currentAngle,
            targetAngle = targetAngle,
            isInDeadZone = isInDeadZone,
            success = true
        )
    }

    /**
     * Command to aim the turret at the Limelight target.
     * This is a one-shot command - it calculates once and sets the target.
     * 
     * For continuous tracking, call this command repeatedly in a loop,
     * or use trackTarget() which is designed for that purpose.
     * 
     * @param limelight The limelight subsystem for target detection
     * @param turret The turret subsystem to control
     * @return Closure that aims the turret
     */
    fun aimTurret(limelight: LimelightSubsystem, turret: TurretSubsystem): Closure {
        return exec {
            val aimResult = calculateTurretAngle(limelight, turret)
            if (aimResult.success) {
                // setAngle will handle clamping to valid range if in dead zone
                turret.setAngle(aimResult.targetAngle)
            }
        }
    }

    /**
     * Command to aim the turret at a specific offset from current position.
     * Useful for testing without Limelight.
     * 
     * @param offsetDegrees Offset from current position in degrees
     * @param turret The turret subsystem to control
     * @return Closure that adjusts the turret
     */
    fun aimTurretByOffset(offsetDegrees: Double, turret: TurretSubsystem): Closure {
        return exec {
            val aimResult = calculateTurretAngleForOffset(offsetDegrees, turret)
            turret.setAngle(aimResult.targetAngle)
        }
    }

    /**
     * Command for continuous target tracking.
     * Call this in a loop to continuously update the turret position
     * based on Limelight target detection.
     * 
     * This is essentially the same as aimTurret() but semantically indicates
     * it should be called repeatedly.
     * 
     * Example usage in TeleOp:
     * ```
     * schedule(loop({ inLoop && trackingEnabled }, exec {
     *     TurretCommands.trackTarget(limelight, turret)
     * }))
     * ```
     * 
     * @param limelight The limelight subsystem for target detection
     * @param turret The turret subsystem to control
     * @return Closure that updates turret tracking
     */
    fun trackTarget(limelight: LimelightSubsystem, turret: TurretSubsystem): Closure {
        return exec {
            val aimResult = calculateTurretAngle(limelight, turret)
            if (aimResult.success) {
                turret.setAngle(aimResult.targetAngle)
            }
            // If no target, hold current position (don't change anything)
        }
    }

    /**
     * Command to center the turret (return to home position).
     * 
     * @param turret The turret subsystem to control
     * @return Closure that centers the turret
     */
    fun centerTurret(turret: TurretSubsystem): Closure {
        return exec {
            turret.setAngle(TurretSubsystem.DEFAULT_ANGLE)
        }
    }

    /**
     * Check if the Limelight currently has a valid target.
     * 
     * @param limelight The limelight subsystem
     * @return true if a target is visible
     */
    fun hasTarget(limelight: LimelightSubsystem): Boolean {
        return limelight.hasTarget()
    }
}
