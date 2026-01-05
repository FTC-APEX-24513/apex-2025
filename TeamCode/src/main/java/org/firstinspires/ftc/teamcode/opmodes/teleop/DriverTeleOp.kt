package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.scope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.continuations.Fiber
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.constants.Alliance
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.physics.ShootingCalculator
import kotlin.math.abs

/**
 * Main TeleOp for DECODE competition.
 *
 * Controls:
 * - Left Trigger: Intake on/off
 * - Right Trigger: Auto-aimed shot
 * - Left/Right Bumpers: Spindexer rotate
 * - D-Pad Left: Toggle field-relative drive
 * - D-Pad Up/Down (Init): Alliance selection
 * - Thumbsticks: Drive
 */
@Suppress("UNUSED")
val driverTeleOp = Mercurial.teleop {
    telemetry.addData("Was I alive?", "I WAS ALIVE!!!!!")
    telemetry.update()
    lateinit var container: HardwareContainer
    try {
        container = HardwareContainer::class.create(hardwareMap)
    } catch (e: Exception) {
        telemetry.addData("ERROR", "Failed to create HardwareContainer: ${e.message}")
        telemetry.update()
    }
    telemetry.addData("Did I create a container?", "I CREATED A CONTAINER!!!!!")
    telemetry.update()

    var alliance = Alliance.RED
    var fieldRelative = RobotConstants.FIELD_RELATIVE_ENABLED_DEFAULT
    val loopTimer = ElapsedTime()

    telemetry.addData("Did I do stuff after the container?", "YES!!!!! Here: Alliance:${alliance.name}, fieldRelative: ${fieldRelative}, alive for ${loopTimer.seconds()}s")
    telemetry.update()

    // ═══════════════════════════════════════════════════════
    // ALLIANCE SELECTION
    // ═══════════════════════════════════════════════════════

    telemetry.addData("gearing up for alliance selection...", "GETTING READY!!!!!")
    telemetry.update()

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
            wait { inInit },
            loop(exec {
                telemetry.addLine("╔═══ ALLIANCE SELECTION ═══╗")
                telemetry.addData("Selected", if (alliance == Alliance.BLUE) "🔵 BLUE" else "🔴 RED")
                telemetry.addLine("D-PAD: ⬆️ Blue | ⬇️ Red")
                telemetry.update()
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
    // MAIN DRIVE LOOP
    // ═══════════════════════════════════════════════════════

    schedule(
        sequence(
            wait { inLoop },
            loop(exec {
                // Update Limelight with robot orientation for MegaTag2
                container.limelight.updateRobotOrientation(container.drive.getHeadingDegrees())
                
                val deadzone = RobotConstants.DRIVE_DEADZONE
                val axial = if (abs(gamepad1.left_stick_y) > deadzone) -gamepad1.left_stick_y.toDouble() else 0.0
                val lateral = if (abs(gamepad1.left_stick_x) > deadzone) gamepad1.left_stick_x.toDouble() else 0.0
                val yaw = if (abs(gamepad1.right_stick_x) > deadzone) gamepad1.right_stick_x.toDouble() else 0.0

                container.drive.drive(axial, lateral, yaw, fieldRelative)

                val loopMs = loopTimer.milliseconds()
                loopTimer.reset()

                // Enhanced telemetry with MegaTag2 data
                val pose = container.getRobotPose()
                val distance = container.getGoalDistance(alliance)
                val currentRPM = container.outtake.getCurrentRPM()

                telemetry.addData("Alliance", if (alliance == Alliance.BLUE) "🔵" else "🔴")
                telemetry.addData("Drive", if (fieldRelative) "Field" else "Robot")
                
                if (pose != null) {
                    telemetry.addData("Pos", "X:%.2f Y:%.2f".format(pose.position.x, pose.position.y))
                } else {
                    telemetry.addData("Pos", "No MegaTag")
                }
                
                if (distance != null) {
                    val targetRPM = ShootingCalculator.calculateShotParameters(distance)?.targetRPM ?: 0.0
                    telemetry.addData("Goal", "%.1fm | %dRPM".format(distance, targetRPM.toInt()))
                }
                
                telemetry.addData("Flywheel", "%dRPM".format(currentRPM.toInt()))
                telemetry.addData("Loop", "%.1fms (%.0fHz)".format(loopMs, 1000.0 / loopMs))
                telemetry.update()
            })
        )
    )

    // ═══════════════════════════════════════════════════════
    // CONTROL BINDINGS
    // ═══════════════════════════════════════════════════════

    // Intake - Left Trigger
    bindSpawn(risingEdge { gamepad1.left_trigger > RobotConstants.INTAKE_TRIGGER_THRESHOLD }, container.intake.collect())
    bindSpawn(risingEdge { gamepad1.left_trigger < RobotConstants.INTAKE_TRIGGER_THRESHOLD }, container.intake.stop())

    // Single Shot - Right Trigger (with realignment check)
    bindSpawn(risingEdge { gamepad1.right_trigger > 0.5 }, container.shoot(alliance))

    // Continuous Shooting - Triangle/Y Button (hold to shoot)
    bindWhileTrue({ gamepad1.triangle }, container.continuousShoot(alliance))

    // Spindexer - Bumpers
    bindSpawn(risingEdge { gamepad1.right_bumper }, container.spindexer.rotateRight())
    bindSpawn(risingEdge { gamepad1.left_bumper }, container.spindexer.rotateLeft())

    // Field-Relative Toggle - D-Pad Left
    bindSpawn(risingEdge { gamepad1.dpad_left }, exec { fieldRelative = !fieldRelative })

    dropToScheduler()
}
