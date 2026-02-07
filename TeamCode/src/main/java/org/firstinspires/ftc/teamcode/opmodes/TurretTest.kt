package org.firstinspires.ftc.teamcode.opmodes

import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.commands.TurretCommands
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem

/**
 * Test OpMode for the Turret Subsystem.
 * 
 * Controls:
 * - Left Stick X: Manual turret angle adjustment (continuous)
 * - Right Bumper: Stop/hold current position
 * - Left Bumper (hold): Continuous Limelight tracking mode
 * - A Button: One-shot aim at current Limelight target
 * - B Button: Center turret (return to 180 degrees)
 * - D-Pad Left/Right: Preset angle jumps (+/- 45 degrees)
 * - D-Pad Up/Down: Fine adjustment (+/- 5 degrees)
 * 
 * Telemetry shows:
 * - Current encoder angle and voltage
 * - Target angle
 * - Angle error
 * - At target status
 * - Limelight tx value (if available)
 * - Dead zone warnings
 */
@Suppress("UNUSED")
val turretTest = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val turret = container.turret
    val limelight = container.limelight

    var manualAngle = TurretSubsystem.DEFAULT_ANGLE
    var trackingEnabled = false
    var lastAimResult: TurretCommands.TurretAimResult? = null

    waitForStart()

    schedule(exec {
        turret.setAngle(TurretSubsystem.DEFAULT_ANGLE)
        manualAngle = TurretSubsystem.DEFAULT_ANGLE
    })

    schedule(loop({ inLoop }, exec {
        val currentAngle = turret.getCurrentAngle()
        val targetAngle = turret.targetAngle
        val error = turret.getAngleError()

        telemetry.addLine("=== TURRET TEST ===")
        telemetry.addLine("")

        telemetry.addLine("--- Encoder Feedback ---")
        telemetry.addData("Current Angle", "%.1f deg", currentAngle)
        telemetry.addData("Encoder Voltage", "%.3f V", turret.getEncoderVoltage())
        telemetry.addLine("")

        telemetry.addLine("--- Target ---")
        telemetry.addData("Target Angle", "%.1f deg", targetAngle)
        telemetry.addData("Manual Angle", "%.1f deg", manualAngle)
        telemetry.addData("Angle Error", "%.1f deg", error)
        telemetry.addData("At Target?", turret.isAtTarget())
        telemetry.addLine("")

        telemetry.addLine("--- Dead Zone ---")
        telemetry.addData("Min Valid Angle", "%.1f deg", turret.getMinValidAngle())
        telemetry.addData("Max Valid Angle", "%.1f deg", turret.getMaxValidAngle())
        telemetry.addData("Current In Dead Zone?", turret.isInDeadZone(currentAngle))
        telemetry.addData("Target In Dead Zone?", turret.isInDeadZone(targetAngle))
        telemetry.addLine("")

        telemetry.addLine("--- Limelight ---")
        telemetry.addData("Tracking Mode", if (trackingEnabled) "ACTIVE" else "OFF")
        val tx = TurretCommands.getTargetOffset(limelight)
        if (tx != null) {
            telemetry.addData("Target tx", "%.1f deg", tx)
            telemetry.addData("Has Target", "YES")
        } else {
            telemetry.addData("Target tx", "N/A")
            telemetry.addData("Has Target", "NO")
        }
        telemetry.addLine("")

        telemetry.addLine("--- Last Aim Result ---")
        lastAimResult?.let { result ->
            if (result.success) {
                telemetry.addData("Limelight tx", "%.1f deg", result.targetTx)
                telemetry.addData("Calculated Target", "%.1f deg", result.targetAngle)
                telemetry.addData("Would Be In Dead Zone?", result.isInDeadZone)
            } else {
                telemetry.addData("Result", "No target found")
            }
        } ?: telemetry.addData("Result", "Press A to aim")
        telemetry.addLine("")

        telemetry.addLine("--- Controls ---")
        telemetry.addLine("Left Stick X: Manual adjust")
        telemetry.addLine("LB (hold): Track target | A: One-shot aim")
        telemetry.addLine("B: Center | RB: Hold position")
        telemetry.addLine("D-Pad L/R: +/-45 deg | U/D: +/-5 deg")

        telemetry.update()
    }))

    schedule(loop({ inLoop }, exec {
        if (trackingEnabled || gamepad1.left_bumper) return@exec
        val stickX = gamepad1.left_stick_x.toDouble()

        if (kotlin.math.abs(stickX) <= 0.1) return@exec
        manualAngle += stickX * 2.0
        manualAngle = ((manualAngle % 360.0) + 360.0) % 360.0
        turret.setAngle(manualAngle)
    }))

    schedule(loop({ inLoop }, exec {
        trackingEnabled = gamepad1.left_bumper

        if (!trackingEnabled) return@exec
        val aimResult = TurretCommands.calculateTurretAngle(limelight, turret)
        lastAimResult = aimResult
        if (!aimResult.success) return@exec
        turret.setAngle(aimResult.targetAngle)
        manualAngle = aimResult.targetAngle
    }))

    bindSpawn(risingEdge { gamepad1.a }, exec {
        lastAimResult = TurretCommands.calculateTurretAngle(limelight, turret)
        if (!lastAimResult.success) return@exec
        manualAngle = lastAimResult.targetAngle
        turret.setAngle(manualAngle)
    })

    bindSpawn(risingEdge { gamepad1.b }, exec {
        manualAngle = TurretSubsystem.DEFAULT_ANGLE
        turret.setAngle(manualAngle)
    })

    bindSpawn(risingEdge { gamepad1.right_bumper }, exec {
        manualAngle = turret.getCurrentAngle()
        turret.hold()
    })

    bindSpawn(risingEdge { gamepad1.dpad_right }, exec {
        manualAngle = ((manualAngle + 45.0) % 360.0)
        turret.setAngle(manualAngle)
    })
    bindSpawn(risingEdge { gamepad1.dpad_left }, exec {
        manualAngle = ((manualAngle - 45.0 + 360.0) % 360.0)
        turret.setAngle(manualAngle)
    })

    bindSpawn(risingEdge { gamepad1.dpad_up }, exec {
        manualAngle = ((manualAngle + 5.0) % 360.0)
        turret.setAngle(manualAngle)
    })
    bindSpawn(risingEdge { gamepad1.dpad_down }, exec {
        manualAngle = ((manualAngle - 5.0 + 360.0) % 360.0)
        turret.setAngle(manualAngle)
    })

    dropToScheduler()
}
