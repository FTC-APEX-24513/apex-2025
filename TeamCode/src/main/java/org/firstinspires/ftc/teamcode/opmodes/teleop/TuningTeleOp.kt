package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

/**
 * Tuning TeleOp for testing spindexer, outtake, and transfer.
 *
 * Controls:
 * - Right Trigger: Flywheel speed (manual)
 * - A Button: Fire
 * - Left/Right Bumpers (hold): Spindexer rotate continuously
 * - Right Stick Y: Transfer position (-1 to 1 mapped to 0 to 1 servo position)
 */
@Suppress("UNUSED")
val tuningTeleOp = Mercurial.teleop {
    val panels = PanelsTelemetry.telemetry

    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val loopTimer = ElapsedTime()

    waitForStart()
    loopTimer.reset()

    // Main loop
    schedule(
        loop({ inLoop }, exec {
            // Flywheel - Right trigger controls speed
            val triggerValue = gamepad1.right_trigger.toDouble()
            if (triggerValue > 0.05) {
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(triggerValue))
            } else if (container.outtake.state is OuttakeSubsystem.State.ManualPower) {
                container.outtake.setState(OuttakeSubsystem.State.Off)
            }

            // Transfer - Right stick Y (-1 to 1) mapped to servo (0 to 1)
            val stickY = -gamepad1.right_stick_y.toDouble() // Invert so up = higher position
            val servoPosition = (stickY + 1.0) / 2.0 // Map -1..1 to 0..1
            container.transfer.setPositionDirect(servoPosition)

            // Telemetry
            val loopMs = loopTimer.milliseconds()
            loopTimer.reset()

            panels.debug("=== TUNING TELEOP ===")
            panels.addData("Loop", "%.1fms (%.0fHz)".format(loopMs, 1000.0 / loopMs))

            panels.debug("")
            panels.debug("=== INTAKE ===")
            panels.addData("State", container.intake.state)

            panels.debug("")
            panels.debug("=== OUTTAKE ===")
            panels.addData("State", container.outtake.state)
            panels.addData("Current RPM", "%d".format(container.outtake.getCurrentRPM().toInt()))
            panels.addData("Trigger", "%.2f".format(triggerValue))

            panels.debug("")
            panels.debug("=== SPINDEXER ===")
            panels.addData("State", container.spindexer.state)
            panels.addData("Power", RobotConstants.SPINDEXER_ROTATION_POWER)

            panels.debug("")
            panels.debug("=== TRANSFER ===")
            panels.addData("Stick Y", "%.2f".format(stickY))
            panels.addData("Servo Pos", "%.4f".format(servoPosition))
            panels.addData("Target Pos", "%.4f".format(container.transfer.getTargetPosition()))

            panels.debug("")
            panels.debug("=== CONTROLS ===")
            panels.debug("RT: Flywheel | A: Fire")
            panels.debug("Bumpers: Spindexer")
            panels.debug("R Stick Y: Transfer")

            panels.update(telemetry)
        })
    )

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

    // Spindexer - Bumpers (hold to continuously rotate)
    bindWhileTrue({ gamepad1.right_bumper }, container.spindexer.continuousRight())
    bindWhileTrue({ gamepad1.left_bumper }, container.spindexer.continuousLeft())
    
    // Stop spindexer when bumpers released
    bindSpawn(risingEdge { !gamepad1.right_bumper }, container.spindexer.stop())
    bindSpawn(risingEdge { !gamepad1.left_bumper }, container.spindexer.stop())

    dropToScheduler()
}
