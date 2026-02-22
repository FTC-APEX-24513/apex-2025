package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.hardware.Gamepad
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.ifHuh
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.scope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Fiber
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.enums.Alliance
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

/**
 * Main Driver TeleOp mode.
 *
 * Controls
 * --------
 * Left stick          — Drive (axial / lateral)
 * Right stick X       — Yaw
 *
 * Right bumper hold   — Intake collect
 * Right bumper release— Intake stop
 *
 * Left bumper hold    — Outtake flywheel on (left trigger sets RPM)
 * Left bumper release — Outtake flywheel off
 *
 * Left trigger        — Variable outtake RPM (0–MAX_RPM) while left bumper held
 * Right trigger       — Intake kick-out (eject)
 *
 * Dpad up             — Hood angle increase (always active, flywheel need not be on)
 * Dpad down           — Hood angle decrease (always active, flywheel need not be on)
 * Right bumper        — Cycle through preset hood angles (always active)
 * Dpad left           — Turret left
 * Dpad right          — Turret right
 *
 * Triangle            — Transfer sequence (all three: 0, 1, 2)
 * Square              — Transfer index 0
 * Cross               — Transfer index 1
 * Circle              — Transfer index 2
 */
@Suppress("UNUSED")
val driverTeleOp = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    var alliance = Alliance.BLUE

    // --- Init: alliance selection ---
    schedule(scope {
        var upFiber by variable<Fiber?> { null }
        var downFiber by variable<Fiber?> { null }

        sequence(exec {
            if (alliance == Alliance.BLUE) {
                gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
                gamepad2.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
            } else {
                gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
                gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
            }
            upFiber = bindSpawn(risingEdge { gamepad1.dpad_up }, exec {
                alliance = Alliance.BLUE
                gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
                gamepad2.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
            })
            downFiber = bindSpawn(risingEdge { gamepad1.dpad_down }, exec {
                alliance = Alliance.RED
                gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
                gamepad2.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
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

    // --- Drivetrain ---
    schedule(
        loop({ inLoop }, exec {
            val axial = gamepad1.left_stick_y.toDouble()
            val lateral = gamepad1.left_stick_x.toDouble()
            val yaw = gamepad1.right_stick_x.toDouble()
            container.follower.setTeleOpDrive(-axial, -lateral, -yaw, true)
        })
    )

    // --- Intake ---
    // Right bumper rising  → collect; falling → stop
    bindSpawn(risingEdge { gamepad1.right_bumper }, container.intake.collect())
    bindSpawn(risingEdge { !gamepad1.right_bumper }, container.intake.stop())

    // Right trigger → kick-out (eject); release → stop
    bindSpawn(risingEdge { gamepad1.right_trigger > 0.1 }, container.intake.eject())
    bindSpawn(risingEdge { gamepad1.right_trigger <= 0.1 }, container.intake.stop())

    // --- Outtake flywheel + hood ---
    var hoodAngle = OuttakeSubsystem.DEFAULT_HOOD_ANGLE

    val setPositions = arrayOf(20.0, 30.0, 40.0, 45.0)
    var setIndex = 0;

    // Right bumper cycles through preset hood angles (works whether or not flywheel is on).
    bindSpawn(risingEdge { gamepad2.right_bumper }, exec {
        hoodAngle = setPositions[setIndex++ % setPositions.size]
    })

    // Hood loop — runs every tick unconditionally.
    // Dpad up/down adjusts the angle; adjustHood sends it to the subsystem regardless of flywheel state.
    schedule(
        loop(
            { inLoop }, sequence(
                exec {
                    if (gamepad2.dpad_up) {
                        hoodAngle =
                            (hoodAngle + 4.0).coerceIn(OuttakeSubsystem.MIN_HOOD_ANGLE, OuttakeSubsystem.MAX_HOOD_ANGLE)
                    } else if (gamepad2.dpad_down) {
                        hoodAngle =
                            (hoodAngle - 4.0).coerceIn(OuttakeSubsystem.MIN_HOOD_ANGLE, OuttakeSubsystem.MAX_HOOD_ANGLE)
                    }
                },
                container.outtake.adjustHood { hoodAngle }
            )
        )
    )

    // Flywheel loop — left bumper spins up (right trigger scales RPM), release stops.
    // Hood angle is also passed to spinUp so the Spinning state stays in sync.
    schedule(
        loop(
            { inLoop },
            ifHuh(
                { gamepad2.left_bumper },
                container.outtake.spinUp(
                    { OuttakeSubsystem.MAX_RPM * (gamepad2.right_trigger.toDouble().takeIf { it != 0.0 } ?: 1.0) },
                    { hoodAngle }
                )
            ).elseHuh(
                container.outtake.stop()
            )
        )
    )

    // --- Turret ---
    // Dpad left/right: drive turret; release → hold
    bindSpawn(risingEdge { gamepad2.dpad_left }, container.turret.drive(-1.0))
    bindSpawn(risingEdge { !gamepad2.dpad_left }, container.turret.hold())
    bindSpawn(risingEdge { gamepad2.dpad_right }, container.turret.drive(1.0))
    bindSpawn(risingEdge { !gamepad2.dpad_right }, container.turret.hold())

    // --- Transfer ---
    // Triangle → sequence all three hoppers (0, 1, 2)
    bindSpawn(risingEdge { gamepad2.triangle }, container.transfer.triggerSequence(arrayOf(0, 1, 2)))
    // Square → hopper 0
    bindSpawn(risingEdge { gamepad2.square }, container.transfer.trigger(0))
    // Cross → hopper 1
    bindSpawn(risingEdge { gamepad2.cross }, container.transfer.trigger(1))
    // Circle → hopper 2
    bindSpawn(risingEdge { gamepad2.circle }, container.transfer.trigger(2))

    // --- Telemetry ---
    schedule(loop({ inLoop }, exec {
        telemetry.addLine("=== INDEXER  [△=all  □=0  ✕=1  ○=2] ===")
        val inv = container.indexer.inventory
        telemetry.addData("[□] Slot 0", inv[0])
        telemetry.addData("[✕] Slot 1", inv[1])
        telemetry.addData("[○] Slot 2", inv[2])

        telemetry.addLine("=== OUTTAKE ===")
        telemetry.addData("Target RPM", "%.0f", container.outtake.targetRPM)
        telemetry.addData("Actual RPM", "%.0f", container.outtake.getCurrentRPM())
        telemetry.addData("Hood angle", "%.1f°", hoodAngle)
        telemetry.addData("At RPM?", container.outtake.isAtTargetRPM())

        telemetry.addLine("=== TURRET ===")
        telemetry.addData("Position", "%.1f°", container.turret.estimatedPosition)
        telemetry.addData("At limit?", container.turret.isAtLimit)

        telemetry.addLine("=== INTAKE ===")
        val rightTrigger = gamepad1.right_trigger
        val rightBumper = gamepad1.right_bumper
        val intakeState = when {
            rightTrigger > 0.1 -> "EJECTING"
            rightBumper -> "COLLECTING"
            else -> "idle"
        }
        telemetry.addData("State", intakeState)

        telemetry.update()
    }))

    dropToScheduler()
}