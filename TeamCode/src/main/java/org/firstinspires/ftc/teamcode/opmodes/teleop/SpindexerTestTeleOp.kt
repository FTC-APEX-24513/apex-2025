package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

/**
 * Test TeleOp for Spindexer and Intake subsystems.
 *
 * Controls (similar to main TeleOp):
 * 
 * - Right Bumper: Intake + auto advance spindexer
 * - Right Trigger: Intake only (surgical tubes)
 * 
 * - Left Trigger: Eject
 * 
 * - D-Pad Right: Spindexer rotate right (+60°)
 * - D-Pad Left: Spindexer rotate left (-60°)
 * - D-Pad Up: Next position in current mode
 * - D-Pad Down: Stop spindexer
 * 
 * - Square (X): Toggle mode (Intake <-> Outtake)
 * - Triangle (Y): Next intake position
 * - Circle (B): Next outtake position
 * - X (A): Kick out / Eject
 * 
 * - Left Stick Y: Manual spindexer power
 * 
 * Controller LED:
 * - Indigo: Intake mode
 * - Orange: Outtake mode
 */
@Suppress("UNUSED")
val spindexerTestTeleOp = Mercurial.teleop {
    val telemetryA = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val loopTimer = ElapsedTime()

    waitForStart()
    loopTimer.reset()

    // ═══════════════════════════════════════════════════════
    // MAIN LOOP - Telemetry & Manual Control
    // ═══════════════════════════════════════════════════════

    schedule(
        loop({ inLoop }, exec {
            val loopMs = loopTimer.milliseconds()
            loopTimer.reset()

            // Manual spindexer control with left stick
            val manualPower = -gamepad1.left_stick_y.toDouble()
            if (kotlin.math.abs(manualPower) > 0.1) {
                container.spindexer.setManualPower(manualPower)
            }

            // ═══════════════════════════════════════════════════════
            // CONTROLLER LED - Mode Indicator
            // ═══════════════════════════════════════════════════════
            
            when (container.spindexer.mode) {
                SpindexerSubsystem.Mode.INTAKE -> {
                    // Indigo for intake
                    gamepad1.setLedColor(0.29, 0.0, 0.51, Gamepad.LED_DURATION_CONTINUOUS)
                }
                SpindexerSubsystem.Mode.OUTTAKE -> {
                    // Orange for outtake
                    gamepad1.setLedColor(1.0, 0.5, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
                }
            }

            // ═══════════════════════════════════════════════════════
            // TELEMETRY
            // ═══════════════════════════════════════════════════════

            telemetryA.addLine("=== SPINDEXER TEST ===")
            telemetryA.addData("Loop", "%.1fms (%.0fHz)".format(loopMs, 1000.0 / loopMs))

            // === SPINDEXER ===
            telemetryA.addLine("")
            telemetryA.addLine("=== SPINDEXER ===")
            telemetryA.addData("Mode", container.spindexer.mode.name)
            telemetryA.addData("State", container.spindexer.state::class.simpleName ?: "Unknown")
            
            // Show target if moving
            val state = container.spindexer.state
            if (state is SpindexerSubsystem.State.MovingToPosition) {
                telemetryA.addData("Target", "%.1f deg".format(state.targetDegrees))
            }
            
            telemetryA.addData("Current Angle", "%.1f deg".format(container.spindexer.getCurrentAngle()))
            telemetryA.addData("Error", "%.2f deg".format(container.spindexer.currentError))
            telemetryA.addData("Power", "%.3f".format(container.spindexer.currentPower))
            telemetryA.addData("Position Index", container.spindexer.currentPositionIndex)
            telemetryA.addData("Position Type", when {
                container.spindexer.isAtIntakePosition() -> "INTAKE"
                container.spindexer.isAtOuttakePosition() -> "OUTTAKE"
                else -> "Between positions"
            })
            telemetryA.addData("At Target", container.spindexer.isAtTarget())
            
            // === ENCODER DIAGNOSTICS ===
            telemetryA.addLine("")
            telemetryA.addLine("=== ENCODER ===")
            telemetryA.addData("Polling Rate", "%.0f Hz".format(container.spindexer.encoderPollingRateHz))

            // Position reference
            telemetryA.addLine("")
            telemetryA.addLine("Position Reference:")
            telemetryA.addLine("0=0° 1=60° 2=120° 3=180° 4=240° 5=300°")
            telemetryA.addLine("Even=Intake, Odd=Outtake")

            // === INTAKE ===
            telemetryA.addLine("")
            telemetryA.addLine("=== INTAKE ===")
            telemetryA.addData("State", container.intake.state::class.simpleName ?: "Unknown")

            telemetryA.update()
        })
    )


    // ═══════════════════════════════════════════════════════
    // LEFT SIDE
    // ═══════════════════════════════════════════════════════

    // Left Trigger
    bindSpawn(
        risingEdge { gamepad1.right_trigger > 0.1 },
        container.intake.collect()
    )
    bindSpawn(
        risingEdge { gamepad1.right_trigger < 0.1 },
        container.intake.stop()
    )

    bindSpawn(risingEdge { gamepad1.right_bumper }, container.intake.eject())
    bindSpawn(risingEdge { !gamepad1.right_bumper }, container.intake.stop())

    // ═══════════════════════════════════════════════════════
    // D-PAD - SPINDEXER DIRECT CONTROL
    // ═══════════════════════════════════════════════════════

    // D-Pad Right - Rotate right (+60°)
    bindSpawn(risingEdge { gamepad1.dpad_right }, container.spindexer.rotateRight())

    // D-Pad Left - Rotate left (-60°)
    bindSpawn(risingEdge { gamepad1.dpad_left }, container.spindexer.rotateLeft())

    // D-Pad Down - Stop
    bindSpawn(risingEdge { gamepad1.cross }, container.spindexer.stop())

    // ═══════════════════════════════════════════════════════
    // FACE BUTTONS
    // ═══════════════════════════════════════════════════════

    // Square (X) - Toggle mode
    bindSpawn(risingEdge { gamepad1.square }, container.spindexer.toggleMode())

    dropToScheduler()
}
