package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

/**
 * Spindexer Time-Based Rotation Test TeleOp
 *
 * This teleop tests the time-based spindexer rotation system.
 * Tune FULL_ROTATION_TIME_MS and SERVO_POWER in FTC Dashboard.
 *
 * Controls:
 * - D-Pad Right: Rotate clockwise 60° (time-based)
 * - D-Pad Left: Rotate counter-clockwise 60° (time-based)
 * - Circle (B): Stop spindexer
 *
 * Telemetry shows:
 * - Current state
 * - Current angle
 * - Tunable parameters (FULL_ROTATION_TIME_MS, SERVO_POWER)
 * - Loop time
 */
@Suppress("UNUSED")
val spindexerTimedTestTeleOp = Mercurial.teleop {
    val telemetryA = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val loopTimer = ElapsedTime()

    waitForStart()
    loopTimer.reset()

    // ═══════════════════════════════════════════════════════
    // MAIN LOOP
    // ═══════════════════════════════════════════════════════

    schedule(
        loop({ inLoop }, exec {
            val loopMs = loopTimer.milliseconds()
            loopTimer.reset()

            // ═══════════════════════════════════════════════════════
            // TELEMETRY
            // ═══════════════════════════════════════════════════════

            telemetryA.addLine("=== SPINDEXER TIME-BASED TEST ===")
            telemetryA.addLine("")

            // Loop performance
            telemetryA.addData("Loop", "%.1fms (%.0fHz)".format(loopMs, 1000.0 / loopMs))
            telemetryA.addLine("")

            // Spindexer state
            telemetryA.addLine("=== SPINDEXER ===")
            telemetryA.addData("State", container.spindexer.state::class.simpleName ?: "Unknown")
            telemetryA.addData("Angle", "%.1f deg".format(container.spindexer.getCurrentAngle()))
            telemetryA.addLine("")

            // Tunable parameters (shown for reference)
            telemetryA.addLine("=== TUNABLE PARAMS (FTC Dashboard) ===")
            telemetryA.addData("FULL_ROTATION_TIME_MS", "%.0f".format(SpindexerSubsystem.FULL_ROTATION_TIME_MS))
            telemetryA.addData("SERVO_POWER", "%.2f".format(SpindexerSubsystem.SERVO_POWER))
            telemetryA.addData("60° Time", "%.0fms".format(SpindexerSubsystem.FULL_ROTATION_TIME_MS / 6.0))
            telemetryA.addLine("")

            // Controls reminder
            telemetryA.addLine("=== CONTROLS ===")
            telemetryA.addLine("D-Pad Right: Rotate CW 60°")
            telemetryA.addLine("D-Pad Left: Rotate CCW 60°")
            telemetryA.addLine("Circle (B): Stop")
            telemetryA.addLine("")
            telemetryA.addLine("Tune params in FTC Dashboard:")
            telemetryA.addLine("- SpindexerSubsystem.FULL_ROTATION_TIME_MS")
            telemetryA.addLine("- SpindexerSubsystem.SERVO_POWER")

            telemetryA.update()
        })
    )

    // ═══════════════════════════════════════════════════════
    // D-PAD CONTROLS
    // ═══════════════════════════════════════════════════════

    bindSpawn(
        risingEdge { gamepad1.dpad_right },
        sequence(
            container.transfer.reset(),
            container.spindexer.rotateRight()
        )
    )

    bindSpawn(
        risingEdge { gamepad1.dpad_left },
        sequence(
            container.transfer.reset(),
            container.spindexer.rotateLeft()
        )
    )

    // ═══════════════════════════════════════════════════════
    // EMERGENCY STOP
    // ═══════════════════════════════════════════════════════

    bindSpawn(
        risingEdge { gamepad1.circle },
        container.spindexer.stop()
    )

    dropToScheduler()
}
