package org.firstinspires.ftc.teamcode.opmodes

import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.commands.OuttakeCommands
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

/**
 * Test OpMode for the Outtake Subsystem.
 * 
 * Controls:
 * - Right Trigger: Increase RPM (proportional)
 * - Left Trigger: Decrease RPM (proportional) 
 * - Right Stick Y: Adjust hood angle (up = higher angle, down = lower)
 * - Right Bumper: Stop everything
 * - Left Bumper: Auto-aim test (uses Limelight + ShooterUltimate)
 * - D-Pad Up/Down: Preset test distances for auto-aim (without Limelight)
 * 
 * Telemetry shows:
 * - Target and current RPM for both motors
 * - Hood angle
 * - Motor power levels
 * - Limelight distance and shot solution (when available)
 */
@Suppress("UNUSED")
val outtakeTest = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val outtake = container.outtake
    val limelight = container.limelight

    var manualRPM = 0.0
    var manualHoodAngle = OuttakeSubsystem.DEFAULT_HOOD_ANGLE
    var lastAimResult: OuttakeCommands.AimResult? = null
    var testDistance = 3.0

    waitForStart()

    schedule(loop({ inLoop }, exec {
        val rpmDelta = (gamepad1.right_trigger - gamepad1.left_trigger) * 100.0
        manualRPM = (manualRPM + rpmDelta).coerceIn(0.0, OuttakeSubsystem.MAX_RPM)

        val angleDelta = -gamepad1.right_stick_y * 1.0
        manualHoodAngle = (manualHoodAngle + angleDelta).coerceIn(
            OuttakeSubsystem.MIN_HOOD_ANGLE,
            OuttakeSubsystem.MAX_HOOD_ANGLE
        )

        telemetry.addLine("=== OUTTAKE TEST ===")
        telemetry.addLine("")
        
        telemetry.addLine("--- Manual Control ---")
        telemetry.addData("Target RPM", "%.0f", manualRPM)
        telemetry.addData("Hood Angle", "%.1f deg", manualHoodAngle)
        telemetry.addLine("")
        
        telemetry.addLine("--- Motor Feedback ---")
        telemetry.addData("Current RPM (avg)", "%.0f", outtake.getCurrentRPM())
        telemetry.addData("Motor 0 RPM", "%.0f", outtake.getMotor0RPM())
        telemetry.addData("Motor 1 RPM", "%.0f", outtake.getMotor1RPM())
        telemetry.addData("At Target?", outtake.isAtTargetRPM())
        telemetry.addLine("")

        val distance = OuttakeCommands.getDistanceToGoal(limelight)
        telemetry.addLine("--- Limelight ---")
        if (distance != null) {
            telemetry.addData("Distance to Goal", "%.2f m", distance)
        } else {
            telemetry.addData("Distance to Goal", "N/A (no pose)")
        }
        telemetry.addLine("")

        telemetry.addLine("--- Auto-Aim Result ---")
        telemetry.addData("Test Distance", "%.2f m (D-Pad Up/Down)", testDistance)
        lastAimResult?.let { result ->
            if (result.success) {
                telemetry.addData("Solved Distance", "%.2f m", result.distance)
                telemetry.addData("Solved Angle", "%d deg", result.angle)
                telemetry.addData("Solved RPM", "%.0f", result.rpm)
                telemetry.addData("Solved Velocity", "%.2f m/s", result.velocity)
            } else {
                telemetry.addData("Result", "No solution found")
            }
        } ?: telemetry.addData("Result", "Press LB for auto-aim")
        telemetry.addLine("")

        telemetry.addLine("--- Controls ---")
        telemetry.addLine("RT/LT: Increase/Decrease RPM")
        telemetry.addLine("Right Stick Y: Hood angle")
        telemetry.addLine("RB: Stop | LB: Auto-aim (Limelight)")
        telemetry.addLine("D-Pad Up/Down: Test distance +/- 0.5m")
        telemetry.addLine("A: Test auto-aim at test distance")

        telemetry.update()
    }))

    schedule(loop({ inLoop }, exec {
        if (!gamepad1.left_bumper && !gamepad1.a) {
            outtake.spinUp(manualRPM, manualHoodAngle)
        }
    }))

    bindSpawn(risingEdge { gamepad1.right_bumper }, exec {
        manualRPM = 0.0
        manualHoodAngle = OuttakeSubsystem.DEFAULT_HOOD_ANGLE
        outtake.stop()
    })

    bindSpawn(risingEdge { gamepad1.left_bumper }, exec {
        lastAimResult = OuttakeCommands.calculateShot(limelight)
        if (lastAimResult?.success == true) {
            manualRPM = lastAimResult!!.rpm
            manualHoodAngle = lastAimResult!!.angle.toDouble()
            outtake.spinUp(manualRPM, manualHoodAngle)
        }
    })

    bindSpawn(risingEdge { gamepad1.dpad_up }, exec {
        testDistance = (testDistance + 0.5).coerceAtMost(10.0)
    })
    bindSpawn(risingEdge { gamepad1.dpad_down }, exec {
        testDistance = (testDistance - 0.5).coerceAtLeast(1.0)
    })

    bindSpawn(risingEdge { gamepad1.a }, exec {
        lastAimResult = OuttakeCommands.calculateShotForDistance(testDistance)
        if (lastAimResult?.success == true) {
            manualRPM = lastAimResult!!.rpm
            manualHoodAngle = lastAimResult!!.angle.toDouble()
            outtake.spinUp(manualRPM, manualHoodAngle)
        }
    })

    dropToScheduler()
}
