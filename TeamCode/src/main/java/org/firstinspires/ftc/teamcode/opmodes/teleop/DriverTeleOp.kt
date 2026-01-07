package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.scope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Fiber
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.constants.Alliance
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.physics.ShootingCalculator
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import kotlin.math.abs

/**
 * Main TeleOp for DECODE competition.
 *
 * Controls:
 * - Left Stick: Drive (forward/strafe)
 * - Right Stick X: Turn
 * - Left Trigger: Intake collect
 * - Right Trigger: Flywheel speed (manual)
 * - A Button: Fire
 * - B Button: Transfer to transfer position
 * - X Button: Reset transfer to default
 * - Y Button (hold): Adjust transfer up
 * - D-Pad Up/Down (Init): Alliance selection
 * - D-Pad Up/Down (Loop): Adjust transfer position
 * - Left/Right Bumpers (hold): Spindexer rotate continuously
 */
@Suppress("UNUSED")
val driverTeleOp = Mercurial.teleop {
    val panels = PanelsTelemetry.telemetry

    // Create hardware container
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    var alliance = Alliance.RED
    val loopTimer = ElapsedTime()

    // ═══════════════════════════════════════════════════════
    // ALLIANCE SELECTION (during init).    
    // ═══════════════════════════════════════════════════════

    schedule(scope {
        var upFiber by variable<Fiber?> { null }
        var downFiber by variable<Fiber?> { null }

        sequence(
            exec {
                upFiber = bindSpawn(risingEdge { gamepad1.dpad_up }, exec {
                    alliance = Alliance.BLUE
                    gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
                })
                downFiber = bindSpawn(risingEdge { gamepad1.dpad_down }, exec {
                    alliance = Alliance.RED
                    gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
                })
            },
            loop({ inInit }, exec {
                panels.debug("=== ALLIANCE SELECTION ===")
                panels.addData("Selected", if (alliance == Alliance.BLUE) "BLUE" else "RED")
                panels.debug("D-PAD: Up=Blue | Down=Red")
                
                // === CONFIGURABLE CONSTANTS ===
                panels.debug("")
                panels.debug("=== INTAKE CONSTANTS ===")
                panels.addData("Collect Power", RobotConstants.INTAKE_COLLECT_POWER)
                panels.addData("Eject Power", RobotConstants.INTAKE_EJECT_POWER)
                panels.addData("Trigger Threshold", RobotConstants.INTAKE_TRIGGER_THRESHOLD)
                
                panels.debug("")
                panels.debug("=== OUTTAKE CONSTANTS ===")
                panels.addData("Ticks/Rev", RobotConstants.OUTTAKE_TICKS_PER_REVOLUTION)
                panels.addData("RPM P Gain", RobotConstants.OUTTAKE_RPM_PROPORTIONAL_GAIN)
                panels.addData("Default RPM", RobotConstants.OUTTAKE_DEFAULT_SPINUP_RPM)
                panels.addData("Launch Duration", RobotConstants.OUTTAKE_LAUNCH_DURATION)
                
                panels.debug("")
                panels.debug("=== SPINDEXER CONSTANTS ===")
                panels.addData("Ticks/Pos", RobotConstants.SPINDEXER_TICKS_PER_POSITION)
                panels.addData("Rotation Power", RobotConstants.SPINDEXER_ROTATION_POWER)
                
                panels.debug("")
                panels.debug("=== TRANSFER CONSTANTS ===")
                panels.addData("Default Pos", RobotConstants.TRANSFER_DEFAULT_POSITION)
                panels.addData("Transfer Pos", RobotConstants.TRANSFER_TRANSFER_POSITION)
                panels.addData("Position Inc", RobotConstants.TRANSFER_POSITION_INCREMENT)
                
                panels.debug("")
                panels.debug("=== DRIVE CONSTANTS ===")
                panels.addData("Deadzone", RobotConstants.DRIVE_DEADZONE)
                
                panels.update(telemetry)
            }),
            exec {
                upFiber?.let { Fiber.CANCEL(it) }
                downFiber?.let { Fiber.CANCEL(it) }
            }
        )
    })

    waitForStart()
    loopTimer.reset()

    // ═══════════════════════════════════════════════════════
    // MAIN LOOP
    // ═══════════════════════════════════════════════════════

    schedule(
        loop({ inLoop }, exec {
            // Update limelight with current heading
            container.limelight.updateRobotOrientation(container.drive.getHeadingDegrees())

            // Drive
            val deadzone = RobotConstants.DRIVE_DEADZONE
            val axial = if (abs(gamepad1.left_stick_y) > deadzone) -gamepad1.left_stick_y.toDouble() else 0.0
            val lateral = if (abs(gamepad1.left_stick_x) > deadzone) gamepad1.left_stick_x.toDouble() else 0.0
            val yaw = if (abs(gamepad1.right_stick_x) > deadzone) gamepad1.right_stick_x.toDouble() else 0.0
            container.drive.drive(axial, lateral, yaw)

            // Flywheel - Right trigger controls speed
            val triggerValue = gamepad1.right_trigger.toDouble()
            if (triggerValue > 0.05) {
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(triggerValue))
            } else if (container.outtake.state is OuttakeSubsystem.State.ManualPower) {
                container.outtake.setState(OuttakeSubsystem.State.Off)
            }

            // ═══════════════════════════════════════════════════════
            // TELEMETRY - Panels Compatible
            // ═══════════════════════════════════════════════════════

            val loopMs = loopTimer.milliseconds()
            loopTimer.reset()

            // === GENERAL ===
            panels.debug("=== GENERAL ===")
            panels.addData("Alliance", if (alliance == Alliance.BLUE) "BLUE" else "RED")
            panels.addData("Loop", "%.1fms (%.0fHz)".format(loopMs, 1000.0 / loopMs))

            // === DRIVE ===
            panels.debug("")
            panels.debug("=== DRIVE ===")
            panels.addData("Heading", "%.1f deg".format(container.drive.getHeadingDegrees()))
            panels.addData("Input", "A:%.2f L:%.2f Y:%.2f".format(axial, lateral, yaw))

            // === LIMELIGHT ===
            panels.debug("")
            panels.debug("=== LIMELIGHT ===")
            val pose = container.getRobotPose()
            val distance = container.getGoalDistance(alliance)
            panels.addData("Has Target", container.limelight.hasTarget())
            panels.addData("TX", "%.2f".format(container.limelight.getTx()))
            panels.addData("TY", "%.2f".format(container.limelight.getTy()))
            if (pose != null) {
                panels.addData("Pose X", "%.2f m".format(pose.position.x))
                panels.addData("Pose Y", "%.2f m".format(pose.position.y))
            } else {
                panels.addData("Pose", "No MegaTag")
            }
            if (distance != null) {
                panels.addData("Goal Dist", "%.2f m".format(distance))
                val targetRPM = ShootingCalculator.calculateShotParameters(distance)?.targetRPM ?: 0.0
                panels.addData("Calc RPM", "%d".format(targetRPM.toInt()))
            }

            // === INTAKE ===
            panels.debug("")
            panels.debug("=== INTAKE ===")
            panels.addData("State", container.intake.state)

            // === OUTTAKE / FLYWHEEL ===
            panels.debug("")
            panels.debug("=== OUTTAKE ===")
            panels.addData("State", container.outtake.state)
            panels.addData("Current RPM", "%d".format(container.outtake.getCurrentRPM().toInt()))
            panels.addData("Trigger", "%.2f".format(triggerValue))
            when (val s = container.outtake.state) {
                is OuttakeSubsystem.State.SpinningUp -> panels.addData("Target RPM", "%d".format(s.targetRPM.toInt()))
                is OuttakeSubsystem.State.Ready -> panels.addData("Target RPM", "%d".format(s.targetRPM.toInt()))
                is OuttakeSubsystem.State.ManualPower -> panels.addData("Power", "%.2f".format(s.power))
                else -> {}
            }

            // === SPINDEXER ===
            panels.debug("")
            panels.debug("=== SPINDEXER ===")
            panels.addData("State", container.spindexer.state)

            // === TRANSFER ===
            panels.debug("")
            panels.debug("=== TRANSFER ===")
            panels.addData("State", container.transfer.state)
            panels.addData("Target Pos", "%.4f".format(container.transfer.getTargetPosition()))
            panels.debug("B=Transfer | X=Reset | DPad=Adjust")

            // Send to both Panels and Driver Station
            panels.update(telemetry)
        })
    )

    // ═══════════════════════════════════════════════════════
    // BUTTON BINDINGS
    // ═══════════════════════════════════════════════════════

    // Intake - Left Trigger
    bindSpawn(
        risingEdge { gamepad1.left_trigger > RobotConstants.INTAKE_TRIGGER_THRESHOLD },
        container.intake.collect()
    )
    bindSpawn(
        risingEdge { gamepad1.left_trigger < RobotConstants.INTAKE_TRIGGER_THRESHOLD },
        container.intake.stop()
    )

    // Fire - A Button
    bindSpawn(risingEdge { gamepad1.a }, container.outtake.launch())

    // Spindexer - Bumpers (hold to continuously rotate, stops when released)
    bindWhileTrue({ gamepad1.right_bumper }, container.spindexer.continuousRight())
    bindWhileTrue({ gamepad1.left_bumper }, container.spindexer.continuousLeft())
    
    // Stop spindexer when bumpers released
    bindSpawn(risingEdge { !gamepad1.right_bumper }, container.spindexer.stop())
    bindSpawn(risingEdge { !gamepad1.left_bumper }, container.spindexer.stop())

    // Transfer Controls
    // B - Go to transfer position
    bindSpawn(risingEdge { gamepad1.b }, container.transfer.transfer())
    
    // X - Reset to default position
    bindSpawn(risingEdge { gamepad1.x }, container.transfer.reset())
    
    // D-Pad Up/Down - Fine tune transfer position (for testing)// Intake - Left Trigger
    //    bindSpawn(
    //        risingEdge { gamepad1.left_trigger > RobotConstants.INTAKE_TRIGGER_THRESHOLD },
    //        container.intake.collect()
    //    )
    //    bindSpawn(
    //        risingEdge { gamepad1.left_trigger < RobotConstants.INTAKE_TRIGGER_THRESHOLD },
    //        container.intake.stop()
    //    )
    bindSpawn(
        risingEdge { gamepad1.dpad_up }, 
        container.transfer.adjustPosition(RobotConstants.TRANSFER_POSITION_INCREMENT)
    )
    bindSpawn(
        risingEdge { gamepad1.dpad_down }, 
        container.transfer.adjustPosition(-RobotConstants.TRANSFER_POSITION_INCREMENT)
    )

    dropToScheduler()
}
