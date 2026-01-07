package org.firstinspires.ftc.teamcode.commands

import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.scope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.continuations.Fiber
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.constants.Alliance
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.constants.ShootingConstants
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.physics.ShootingCalculator
import kotlin.math.abs
import kotlin.math.sqrt

// ═══════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

/**
 * Get distance to goal using MegaTag2.
 * Returns null if no AprilTag visible (no fallback).
 */
fun HardwareContainer.getGoalDistance(alliance: Alliance): Double? {
    return limelight.getDistance2DToGoal(alliance)
}

/**
 * Get robot pose from MegaTag2.
 * Returns null if no AprilTag visible.
 */
fun HardwareContainer.getRobotPose(): Pose3D? {
    return limelight.getBotPoseMT2()
}

// ═══════════════════════════════════════════════════════
// ALIGNMENT
// ═══════════════════════════════════════════════════════

fun HardwareContainer.isAligned() =
    !limelight.hasTarget() || abs(limelight.getTx()) < RobotConstants.ALIGNMENT_TOLERANCE

fun HardwareContainer.autoAlign(): Closure = scope {
    var lastError by variable { 0.0 }

    sequence(loop({ !isAligned() }, exec {
        if (!limelight.hasTarget()) return@exec

        val tx = limelight.getTx()
        val turn = -(RobotConstants.ALIGNMENT_KP * tx + RobotConstants.ALIGNMENT_KD * (tx - lastError))
        lastError = tx

        drive.drive(0.0, 0.0, turn)
    }), exec { drive.stop() })
}

// ═══════════════════════════════════════════════════════
// SHOOTING
// ═══════════════════════════════════════════════════════

/**
 * Single intelligent shot with auto-alignment.
 * Checks if robot moved since last shot and realigns if needed.
 */
fun HardwareContainer.shoot(alliance: Alliance): Closure = scope {
    var rpm by variable { 0.0 }
    var dist by variable { 0.0 }
    var lastPose by variable<Pose3D?> { null }
    var spindown by variable<Fiber?> { null }

    sequence(
        // Check if robot moved since last shot
        exec {
            val currentPose = getRobotPose()
            val needsAlignment = if (lastPose != null && currentPose != null) {
                // Check if robot moved significantly
                val dx = currentPose.position.x - lastPose!!.position.x
                val dy = currentPose.position.y - lastPose!!.position.y
                val distance = sqrt(dx * dx + dy * dy)
                distance > RobotConstants.SHOT_RECALC_THRESHOLD
            } else {
                true  // Always align if no previous pose
            }

            if (!needsAlignment) return@exec  // Skip alignment if robot hasn't moved
        },

        // Align to target
        autoAlign(),

        // Calculate distance and RPM
        exec {
            val currentPose = getRobotPose()
            lastPose = currentPose  // Store for next shot

            dist = getGoalDistance(alliance) ?: run {
                // No AprilTag visible - use default RPM
                rpm = ShootingConstants.DEFAULT_SHOOTING_RPM
                return@exec
            }

            rpm = ShootingCalculator.calculateShotParameters(dist)?.targetRPM ?: ShootingConstants.DEFAULT_SHOOTING_RPM
        },

        // Spin up and fire
        outtake.spinToRPM(rpm), wait { outtake.isAtTargetSpeed(rpm) },

        // Auto-spindown after delay
        exec {
            spindown?.let { Fiber.CANCEL(it) }
//            spindown = schedule(
//                sequence(
//                    wait(RobotConstants.FLYWHEEL_SPINDOWN_TIME), outtake.stop(), exec { spindown = null })
//            )
        })
}

/**
 * Continuous shooting - fires repeatedly while button held.
 * Uses physics-based recovery time between shots.
 * No realignment during continuous fire.
 */
fun HardwareContainer.continuousShoot(alliance: Alliance): Closure = scope {
    var rpm by variable { 0.0 }
    var dist by variable { 0.0 }
    var shotCount by variable { 0 }

    sequence(
        // Initial alignment (once)
        autoAlign(),

        // Calculate distance and RPM (once)
        exec {
            dist = getGoalDistance(alliance) ?: run {
                // No AprilTag - use default RPM
                rpm = ShootingConstants.DEFAULT_SHOOTING_RPM
                return@exec
            }

            rpm = ShootingCalculator.calculateShotParameters(dist)?.targetRPM ?: ShootingConstants.DEFAULT_SHOOTING_RPM
        },

        // Continuous firing loop
        loop(
            { true }, sequence(
                // Spin up to target RPM
                outtake.spinToRPM(rpm), wait { outtake.isAtTargetSpeed(rpm) },

                exec { shotCount++ },

                // Wait for physics-based recovery time
                wait(ShootingCalculator.calculateRecoveryTime(rpm))
            )
        ),

        // Cleanup when button released
        exec {
            outtake.stop()
        })
}
