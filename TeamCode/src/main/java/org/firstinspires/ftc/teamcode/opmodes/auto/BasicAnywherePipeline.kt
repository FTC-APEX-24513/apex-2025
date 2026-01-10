package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.hardware.Gamepad
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.constants.Alliance
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.opmodes.teleop.driverTeleOp

/**
 * Basic Anywhere Pipeline Autonomous -> TeleOp
 *
 * This is a minimal autonomous that can be used from any starting position.
 * It simply waits for autonomous to finish, then transitions to driver control.
 *
 * Use this when:
 * - You want to skip autonomous and go straight to teleop
 * - You're testing and don't want to run paths
 * - You need a safe fallback opmode
 *
 * Alliance selection: D-Pad Up = Blue, D-Pad Down = Red (during init)
 */
@Suppress("UNUSED")
val aBasicAnywherePipeline = Mercurial.pipelineAutonomous {

    lateinit var container: HardwareContainer
    lateinit var pathTimer: Timer
    val telemetryA = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    var alliance = Alliance.BLUE
    var pathState = 0

    // Default starting pose (center of field)
    val startPose = Pose(60.0, 60.0, Math.toRadians(0.0))

    fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                // Minimal autonomous - just switch to AprilTag pipeline and finish
                container.limelight.useAprilTagPipeline()
                setPathState(1)
            }
        }
    }

    sequence(
        exec {
            pathTimer = Timer()

            container = HardwareContainer::class.create(hardwareMap, scheduler).also {
                it.startPeriodic()
            }

            container.follower.setStartingPose(startPose)

            // Set initial LED to blue
            gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
        },
        // Alliance selection during init
        loop({ inInit }, exec {
            if (gamepad1.dpad_up) {
                alliance = Alliance.BLUE
                gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
            } else if (gamepad1.dpad_down) {
                alliance = Alliance.RED
                gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
            }

            telemetryA.addLine("=== BASIC ANYWHERE AUTONOMOUS ===")
            telemetryA.addLine("")
            telemetryA.addData("Alliance", if (alliance == Alliance.BLUE) "BLUE" else "RED")
            telemetryA.addLine("")
            telemetryA.addLine("D-Pad Up = Blue")
            telemetryA.addLine("D-Pad Down = Red")
            telemetryA.addLine("")
            telemetryA.addLine("This autonomous does minimal movement")
            telemetryA.addLine("and transitions to teleop immediately.")
            telemetryA.update()
        }),
        wait { inLoop },
        exec {
            setPathState(0)
        },
        loop({ pathState < 1 }, exec {
            container.follower.update()
            autonomousPathUpdate()

            telemetryA.addLine("=== BASIC ANYWHERE AUTONOMOUS ===")
            telemetryA.addData("Alliance", if (alliance == Alliance.BLUE) "BLUE" else "RED")
            telemetryA.addData("State", pathState)
            telemetryA.addLine("")
            telemetryA.addLine("Transitioning to TeleOp...")
            telemetryA.update()
        })
    )

    dropToScheduler()

    driverTeleOp.program
}
