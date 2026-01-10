package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.scope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Fiber
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.commands.getGoalDistance
import org.firstinspires.ftc.teamcode.constants.Alliance
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.physics.ShootingCalculator
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

/**
 * Main TeleOp for DECODE competition.
 *
 * Controls:
 * - Left Stick: Drive (forward/strafe)
 * - Right Stick X: Turn
 *
 * - Right Bumper: Intake (auto-spins spindexer to next intake position)
 * - Right Trigger: Surgical tubes only (intake rollers)
 *
 * - Left Bumper: Limelight outtake (lock RPM from distance)
 * - Left Trigger: Manual flywheel power
 *
 * - D-Pad Up: Manual transfer up
 * - D-Pad Down: Manual transfer down
 * - D-Pad Right: Spindexer rotate right
 * - D-Pad Left: Spindexer rotate left
 *
 * - Triangle (Y): Localize (update pose from AprilTag)
 * - X (A on Xbox): Kick out from spindexer (eject)
 * - Square (X on Xbox): Toggle spindexer mode (Intake <-> Outtake)
 *
 * Controller LED:
 * - Indigo: Intake mode active
 * - Orange: Outtake mode active
 *
 * Vibration: Vibrates when in shooting zone
 *
 * Init Controls:
 * - D-Pad Up/Down: Alliance selection (Blue/Red)
 */
@Suppress("UNUSED")
val driverTeleOp = Mercurial.teleop {
    val telemetryA = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    var alliance = Alliance.BLUE
    val loopTimer = ElapsedTime()

//    val transferDelta = 0.02

    // ═══════════════════════════════════════════════════════
    // ALLIANCE SELECTION (during init)
    // ═══════════════════════════════════════════════════════

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
            telemetryA.addLine("=== ALLIANCE SELECTION ===")
            telemetryA.addData("Selected", if (alliance == Alliance.BLUE) "BLUE" else "RED")
            telemetryA.addLine("D-PAD: Up=Blue | Down=Red")
            telemetryA.update()
        }), exec {
            upFiber?.let { Fiber.CANCEL(it) }
            downFiber?.let { Fiber.CANCEL(it) }
        })
    })

    waitForStart()
    container.follower.startTeleopDrive(true)
    loopTimer.reset()

    // ═══════════════════════════════════════════════════════
    // MAIN LOOP
    // ═══════════════════════════════════════════════════════

    schedule(
        loop({ inLoop }, exec {
            container.follower.update()
            container.limelight.updateRobotOrientation(Math.toDegrees(container.follower.pose.heading))

            // Drive - Left stick for movement, right stick for rotation
            val axial = -gamepad1.left_stick_y.toDouble()
            val lateral = gamepad1.left_stick_x.toDouble()
            val yaw = gamepad1.right_stick_x.toDouble()
            container.follower.setTeleOpDrive(axial, -lateral, -yaw, true)

            // Manual flywheel with left trigger
            val leftTrigger = gamepad1.left_trigger.toDouble()
            if (leftTrigger > 0.05) {
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(leftTrigger))
            }

            // ═══════════════════════════════════════════════════════
            // CONTROLLER LED - Spindexer Mode Indicator
            // ═══════════════════════════════════════════════════════

//            when (container.spindexer.mode) {
//                SpindexerSubsystem.Mode.INTAKE -> {
//                    // Indigo (blue-purple) for intake
//                    gamepad1.setLedColor(0.29, 0.0, 0.51, Gamepad.LED_DURATION_CONTINUOUS)
//                }
//                SpindexerSubsystem.Mode.OUTTAKE -> {
//                    // Orange for outtake
//                    gamepad1.setLedColor(1.0, 0.5, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
//                }
//            }

            // ═══════════════════════════════════════════════════════
            // VIBRATION - Shooting Zone Feedback
            // ═══════════════════════════════════════════════════════

            val distance = container.getGoalDistance(alliance)
            if (distance != null) {
                when {
                    // Optimal zone - strong rumble
                    distance in OuttakeSubsystem.SHOOTING_ZONE_OPTIMAL_MIN..OuttakeSubsystem.SHOOTING_ZONE_OPTIMAL_MAX -> {
                        gamepad1.rumble(0.8, 0.8, 100)
                    }
                    // Good zone - light rumble
                    distance in OuttakeSubsystem.SHOOTING_ZONE_MIN_METERS..OuttakeSubsystem.SHOOTING_ZONE_MAX_METERS -> {
                        gamepad1.rumble(0.3, 0.3, 100)
                    }
                }
            }

            // ═══════════════════════════════════════════════════════
            // TELEMETRY
            // ═══════════════════════════════════════════════════════

            val loopMs = loopTimer.milliseconds()
            loopTimer.reset()

            // === GENERAL ===
            telemetryA.addLine("=== GENERAL ===")
            telemetryA.addData("Alliance", if (alliance == Alliance.BLUE) "BLUE" else "RED")
            telemetryA.addData("Loop", "%.1fms (%.0fHz)".format(loopMs, 1000.0 / loopMs))

            // === VOLTAGE ===
            telemetryA.addLine("")
            telemetryA.addLine("=== VOLTAGE ===")
            val batteryVoltage = container.voltageCompensation.getVoltage()
            val compensationMultiplier = container.voltageCompensation.getCompensationMultiplier()
            telemetryA.addData("Battery", "%.2fV".format(batteryVoltage))
            telemetryA.addData("Compensation", "%.2fx".format(compensationMultiplier))

            // === DRIVE ===
            telemetryA.addLine("")
            telemetryA.addLine("=== DRIVE ===")
            telemetryA.addData("Heading", "%.1f deg".format(Math.toDegrees(container.follower.pose.heading)))

            // === LIMELIGHT & SHOOTING ===
            telemetryA.addLine("")
            telemetryA.addLine("=== SHOOTING ===")
            val shotParams = distance?.let { ShootingCalculator.calculateShotParameters(it) }

            telemetryA.addData("Has Target", container.limelight.hasTarget())
            if (distance != null) {
                telemetryA.addData("Distance", "%.2f m".format(distance))

                // Show shooting zone status
                val zoneStatus = when {
                    distance in OuttakeSubsystem.SHOOTING_ZONE_OPTIMAL_MIN..OuttakeSubsystem.SHOOTING_ZONE_OPTIMAL_MAX -> "OPTIMAL"
                    distance in OuttakeSubsystem.SHOOTING_ZONE_MIN_METERS..OuttakeSubsystem.SHOOTING_ZONE_MAX_METERS -> "GOOD"
                    distance < OuttakeSubsystem.SHOOTING_ZONE_MIN_METERS -> "TOO CLOSE"
                    else -> "TOO FAR"
                }
                telemetryA.addData("Zone", zoneStatus)

                if (shotParams != null) {
                    telemetryA.addData("Calc RPM", "%d".format(shotParams.targetRPM.toInt()))
                } else {
                    telemetryA.addData("Calc RPM", "OUT OF RANGE")
                }
            } else {
                telemetryA.addData("Distance", "No Target")
            }

            // === OUTTAKE / FLYWHEEL ===
            telemetryA.addLine("")
            telemetryA.addLine("=== FLYWHEEL ===")
            telemetryA.addData("State", container.outtake.state::class.simpleName ?: "Unknown")
            telemetryA.addData("Current RPM", "%d".format(container.outtake.getCurrentRPM().toInt()))

            // Motor power and voltage telemetry
            val motorPower = container.outtake.getCurrentMotorPower()
            val effectiveVoltage = motorPower * batteryVoltage
            telemetryA.addData("Motor Power", "%.2f (%.1f%%)".format(motorPower, motorPower * 100.0))
            telemetryA.addData("Effective V", "%.2fV".format(effectiveVoltage))

            val lockedRPM = container.outtake.lockedRPM
            val lockedDist = container.outtake.lockedDistance
            if (lockedRPM != null) {
                telemetryA.addData("Locked RPM", "%d".format(lockedRPM.toInt()))
                if (lockedDist != null) {
                    telemetryA.addData("Locked Dist", "%.2f m".format(lockedDist))
                }
                telemetryA.addData("Ready", if (container.outtake.isReady()) "YES" else "Spinning...")
            } else {
                telemetryA.addData("Locked", "None (Press LB)")
            }

            // === SPINDEXER ===
            telemetryA.addLine("")
            telemetryA.addLine("=== SPINDEXER ===")
            telemetryA.addData("State", container.spindexer.state::class.simpleName ?: "Unknown")
            telemetryA.addData("Angle", "%.1f deg".format(container.spindexer.getCurrentAngle()))

            // === TRANSFER ===
            telemetryA.addLine("")
            telemetryA.addLine("=== TRANSFER ===")
            telemetryA.addData("Position", "%.3f".format(container.transfer.getTargetPosition()))

            // === INTAKE ===
            telemetryA.addLine("")
            telemetryA.addLine("=== INTAKE ===")
            telemetryA.addData("State", container.intake.state::class.simpleName ?: "Unknown")

            // === CONTROLS REMINDER ===
            telemetryA.addLine("")
            telemetryA.addLine("=== CONTROLS ===")
            telemetryA.addLine("RB=Intake+Spin | RT=Tubes | LB=Lock | LT=Manual")
            telemetryA.addLine("DPad=Transfer/Spin | Y=Localize | X=Kick | []]=Mode")

            telemetryA.update()
        })
    )

    // ═══════════════════════════════════════════════════════
    // RIGHT SIDE - INTAKE CONTROLS
    // ═══════════════════════════════════════════════════════

    bindSpawn(
        risingEdge { gamepad1.right_bumper },
        container.intake.collect(),
    )
    bindSpawn(
        risingEdge { !gamepad1.right_bumper }, container.intake.stop()
    )

//    bindSpawn(
//        risingEdge { gamepad1.right_trigger > 0.1 },
//        container.intake.collect()
//    )
//    bindSpawn(
//        risingEdge { gamepad1.right_trigger < 0.1 },
//        container.intake.stop()
//    )

    // ═══════════════════════════════════════════════════════
    // LEFT SIDE - OUTTAKE CONTROLS
    // ═══════════════════════════════════════════════════════

    bindSpawn(risingEdge { gamepad1.left_bumper }, exec {
        val dist = container.getGoalDistance(alliance)
        if (dist != null) {
            container.outtake.lockToDistanceDirect(dist)
        } else {
            container.outtake.spinToRPMDirect(OuttakeSubsystem.DEFAULT_SHOOTING_RPM)
        }
    })

    bindSpawn(risingEdge { gamepad1.left_trigger < 0.05 }, exec {
        if (container.outtake.lockedRPM == null) {
            container.outtake.setState(OuttakeSubsystem.State.Off)
        }
    })

    // ═══════════════════════════════════════════════════════
    // D-PAD CONTROLS
    // ═══════════════════════════════════════════════════════

    bindSpawn(
        risingEdge { gamepad1.dpad_up }, container.transfer.transfer()
    )

    bindSpawn(
        risingEdge { gamepad1.dpad_down }, container.transfer.reset()
    )

    bindSpawn(
        risingEdge { gamepad1.dpad_right }, sequence(
            container.transfer.reset(), container.spindexer.rotateRight()
        )
    )

    bindSpawn(
        risingEdge { gamepad1.dpad_left }, sequence(
            container.transfer.reset(), container.spindexer.rotateLeft()
        )
    )

    // ═══════════════════════════════════════════════════════
    // FACE BUTTON CONTROLS
    // ═══════════════════════════════════════════════════════

    bindSpawn(risingEdge { gamepad1.triangle }, exec {
        val pose = container.limelight.getBotPoseMT2()
        if (pose != null) {
            // TODO: Update Pedro Pathing localization with this pose
            // For now, just confirm we got a pose
            gamepad1.rumble(0.5, 0.5, 200) // Feedback that localization worked
        } else {
            gamepad1.rumble(0.0, 1.0, 500) // Right-side rumble = no target
        }
    })

    // X (A on Xbox) - Kick out / Eject from spindexer
    bindSpawn(
        risingEdge { gamepad1.cross }, container.intake.eject()
    )
    bindSpawn(
        risingEdge { !gamepad1.cross }, container.intake.stop()
    )

    // Square (X on Xbox) - Toggle spindexer mode (Intake <-> Outtake)
//    bindSpawn(
//        risingEdge { gamepad1.square },
//        container.spindexer.toggleMode()
//    )

    // B Button - Stop flywheel / Reset transfer (safety)
    bindSpawn(
        risingEdge { gamepad1.circle }, sequence(
            container.outtake.stop(), container.transfer.reset()
        )
    )

    dropToScheduler()
}
