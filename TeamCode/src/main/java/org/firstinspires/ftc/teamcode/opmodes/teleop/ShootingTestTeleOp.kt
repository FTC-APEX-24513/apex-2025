package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.constants.Alliance
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.physics.ShootingCalculator
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import kotlin.math.PI

/**
 * TeleOp for tuning shooting physics constants.
 *
 * Controls:
 * - Left Stick Y: Drive Forward/Back
 * - Right Stick X: Turn
 *
 * Tuning:
 * - D-Pad Up/Down: Adjust TEST DISTANCE (+/- 0.1m) - Overrides Limelight if set
 * - D-Pad Left/Right: Adjust EFFICIENCY (+/- 0.01)
 * - Triangle / Cross: Adjust LAUNCH ANGLE (+/- 1 degree)
 *
 * Actions:
 * - Right Bumper: SPIN UP (Calculate RPM based on distance/constants)
 * - Left Bumper: FIRE (Spindexer eject)
 * - Left Trigger: STOP Flywheel
 */
@Suppress("UNUSED")
val shootingTestTeleOp = Mercurial.teleop {
    val telemetryA = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    // Test state
    var manualDistance = 1.5 // Start at 1.5m
    var useLimelight = true
    var calculatedRPM = 0.0
    var targetVelocity = 0.0

    val loopTimer = ElapsedTime()

    waitForStart()
    container.follower.startTeleopDrive()
    loopTimer.reset()

    schedule(
        loop({ inLoop }, exec {
            val loopMs = loopTimer.milliseconds()
            loopTimer.reset()

            val axial = -gamepad1.left_stick_y.toDouble()
            val lateral = gamepad1.left_stick_x.toDouble()
            val yaw = gamepad1.right_stick_x.toDouble()
            container.follower.setTeleOpDrive(axial, -lateral, -yaw, true)

            // Update Limelight orientation
            container.limelight.updateRobotOrientation(Math.toDegrees(container.follower.pose.heading))

            // Get Distance (Limelight or Manual)
            val limelightDist = container.limelight.getDistance2DToGoal(Alliance.RED) // Default to RED goal for test
            val activeDistance = if (useLimelight && limelightDist != null) limelightDist else manualDistance

            // Calculate Shot
            val shotParams = ShootingCalculator.calculateShotParameters(activeDistance)
            
            if (shotParams != null) {
                calculatedRPM = shotParams.targetRPM
                targetVelocity = shotParams.requiredVelocity
            } else {
                calculatedRPM = 0.0
                targetVelocity = 0.0
            }

            // Telemetry
            telemetryA.addLine("=== SHOOTING PHYSICS TUNER ===")
            telemetryA.addData("Distance Source", if (useLimelight && limelightDist != null) "Limelight" else "MANUAL")
            telemetryA.addData("Active Distance", "%.2f m".format(activeDistance))
            if (limelightDist != null) {
                telemetryA.addData("Limelight Read", "%.2f m".format(limelightDist))
            }
            
            telemetryA.addLine("")
            telemetryA.addLine("--- TUNABLE CONSTANTS ---")
            telemetryA.addData("Efficiency (L/R)", "%.2f".format(OuttakeSubsystem.FLYWHEEL_TO_BALL_EFFICIENCY))
            telemetryA.addData("Angle (Tri/X)", "%.1f deg".format(Math.toDegrees(OuttakeSubsystem.LAUNCH_ANGLE_RADIANS)))
            
            telemetryA.addLine("")
            telemetryA.addLine("--- CALCULATION RESULTS ---")
            telemetryA.addData("Target Velocity", "%.2f m/s".format(targetVelocity))
            telemetryA.addData("Calculated RPM", "%.0f".format(calculatedRPM))
            
            telemetryA.addLine("")
            telemetryA.addLine("--- REALTIME STATUS ---")
            telemetryA.addData("Flywheel RPM", "%.0f".format(container.outtake.getCurrentRPM()))
            telemetryA.addData("Target RPM", "%.0f".format(container.outtake.lockedRPM ?: 0.0))
            
            telemetryA.update()
        })
    )

    // =================================================================================
    // CONTROLS
    // =================================================================================

    // Distance Adjustment (Manual Override)
    bindSpawn(risingEdge { gamepad1.dpad_up }, exec { 
        manualDistance += 0.1 
        useLimelight = false
    })
    bindSpawn(risingEdge { gamepad1.dpad_down }, exec { 
        manualDistance = (manualDistance - 0.1).coerceAtLeast(0.5) 
        useLimelight = false
    })
    // Reset to Limelight on stick click
    bindSpawn(risingEdge { gamepad1.right_stick_button }, exec { useLimelight = true })

    // Efficiency Tuning
    bindSpawn(risingEdge { gamepad1.dpad_right }, exec { 
        OuttakeSubsystem.FLYWHEEL_TO_BALL_EFFICIENCY = (OuttakeSubsystem.FLYWHEEL_TO_BALL_EFFICIENCY + 0.01).coerceIn(0.1, 1.0)
    })
    bindSpawn(risingEdge { gamepad1.dpad_left }, exec { 
        OuttakeSubsystem.FLYWHEEL_TO_BALL_EFFICIENCY = (OuttakeSubsystem.FLYWHEEL_TO_BALL_EFFICIENCY - 0.01).coerceIn(0.1, 1.0)
    })

    // Angle Tuning
    bindSpawn(risingEdge { gamepad1.triangle }, exec { 
        OuttakeSubsystem.LAUNCH_ANGLE_RADIANS += Math.toRadians(1.0)
    })
    bindSpawn(risingEdge { gamepad1.cross }, exec { 
        OuttakeSubsystem.LAUNCH_ANGLE_RADIANS = (OuttakeSubsystem.LAUNCH_ANGLE_RADIANS - Math.toRadians(1.0)).coerceAtLeast(0.0)
    })

    // Spin Up (Apply Calculated RPM)
    bindSpawn(risingEdge { gamepad1.right_bumper }, exec {
        // Recalculate one last time to be safe
        val dist = if (useLimelight) container.limelight.getDistance2DToGoal(Alliance.RED) ?: manualDistance else manualDistance
        val params = ShootingCalculator.calculateShotParameters(dist)
        if (params != null) {
            container.outtake.spinToRPMDirect(params.targetRPM)
        }
    })

    // Stop Flywheel
    bindSpawn(risingEdge { gamepad1.left_trigger > 0.5 }, container.outtake.stop())

    // Fire (Eject)
    bindSpawn(risingEdge { gamepad1.left_bumper }, container.intake.eject())
    bindSpawn(risingEdge { !gamepad1.left_bumper }, container.intake.stop())

    dropToScheduler()
}