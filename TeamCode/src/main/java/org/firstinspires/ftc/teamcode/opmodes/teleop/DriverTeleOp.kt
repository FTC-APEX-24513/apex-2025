package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.hardware.Gamepad
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.scope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Fiber
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.constants.Alliance
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

/**
 * Main Driver TeleOp mode.
 */
@Suppress("UNUSED")
val driverTeleOp = Mercurial.teleop {
    val telemetry = PanelsTelemetry.telemetry
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    var alliance = Alliance.BLUE

    schedule(scope {
        var upFiber by variable<Fiber?> { null }
        var downFiber by variable<Fiber?> { null }

        sequence(exec {
            if (alliance == Alliance.BLUE) {
                gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
            } else {
                gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
            }
            upFiber = bindSpawn(risingEdge { gamepad1.dpad_up }, exec {
                alliance = Alliance.BLUE
                gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
            })
            downFiber = bindSpawn(risingEdge { gamepad1.dpad_down }, exec {
                alliance = Alliance.RED
                gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
            })
        }, loop({ inInit }, exec {
            telemetry.addLine("=== ALLIANCE SELECTION ===")
            telemetry.addData("Selected", if (alliance == Alliance.BLUE) "BLUE" else "RED")
            telemetry.addLine("D-PAD: Up=Blue | Down=Red")
            telemetry.update()
        }), exec {
            upFiber?.let { Fiber.CANCEL(it) }
            downFiber?.let { Fiber.CANCEL(it) }
        })
    })

    waitForStart()
    container.follower.startTeleopDrive(true)

    schedule(
        loop({ inLoop }, exec {
            val axial = gamepad1.left_stick_y.toDouble()
            val lateral = gamepad1.left_stick_x.toDouble()
            val yaw = gamepad1.right_stick_x.toDouble()
            container.follower.setTeleOpDrive(-axial, -lateral, -yaw, true)
        })
    )

    bindSpawn(risingEdge { gamepad1.right_bumper }, container.intake.collect())
    bindSpawn(risingEdge { !gamepad1.right_bumper }, container.intake.stop())
    bindSpawn(
        risingEdge { gamepad1.left_bumper },
        exec { container.outtake.spinToRPMDirect(OuttakeSubsystem.DEFAULT_SHOOTING_RPM) }
    )
    bindSpawn(risingEdge { gamepad1.left_trigger < 0.05 }, exec {
        if (container.outtake.lockedRPM == null) {
            container.outtake.setState(OuttakeSubsystem.State.Off)
        }
    })

    bindSpawn(risingEdge { gamepad1.dpad_up }, container.transfer.transfer())
    bindSpawn(risingEdge { gamepad1.dpad_down }, container.transfer.reset())
    bindSpawn(
        risingEdge { gamepad1.dpad_right },
        sequence(container.transfer.reset(), container.spindexer.rotateRight())
    )
    bindSpawn(
        risingEdge { gamepad1.dpad_left },
        sequence(container.transfer.reset(), container.spindexer.rotateLeft())
    )

    bindSpawn(risingEdge { gamepad1.cross }, container.intake.eject())
    bindSpawn(risingEdge { !gamepad1.cross }, container.intake.stop())

    bindSpawn(risingEdge { gamepad1.circle }, sequence(container.outtake.stop(), container.transfer.reset()))

    dropToScheduler()
}
