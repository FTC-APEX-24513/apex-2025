package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.commands.getGoalDistance
import org.firstinspires.ftc.teamcode.constants.Alliance
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.opmodes.teleop.driverTeleOp
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

/**
 * Pipeline Autonomous that transitions to TeleOp.
 *
 * This autonomous runs a simple path (score preload), then automatically
 * transitions into the driver-controlled TeleOp when finished.
 *
 * IMPORTANT: To use this legally in competition, the robot must NOT move during init.
 * All autonomous movement must wait for the start button to be pressed.
 *
 * The teleop will appear as "pipelineAutonomous |> teleop (System)" in the OpMode list.
 */
@Suppress("UNUSED")
val pipelineAutonomous = Mercurial.pipelineAutonomous {

    lateinit var container: HardwareContainer
    lateinit var pathTimer: Timer

    var pathState = 0
    var alliance = Alliance.BLUE

    val startPoseBlueBottom = Pose(56.0, 16.0, Math.toRadians(180.0))
    val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    lateinit var blueStartToScore: PathChain

    fun buildPaths() {
        blueStartToScore = container.follower.pathBuilder()
            .addPath(Path(BezierLine(startPoseBlueBottom, blueScore)))
            .setLinearHeadingInterpolation(
                startPoseBlueBottom.heading,
                blueScore.heading
            )
            .build()
    }

    fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                container.intake.collect()
                container.follower.followPath(blueStartToScore)
                setPathState(1)
            }
            1 -> if (!container.follower.isBusy) {
                container.intake.stop()
                container.limelight.useAprilTagPipeline()
                setPathState(2)
            }
        }
    }

    sequence(
        exec {
            pathTimer = Timer()

            container = HardwareContainer::class.create(hardwareMap, scheduler).also {
                it.startPeriodic()
            }

            buildPaths()
            container.follower.setStartingPose(startPoseBlueBottom)
        },
        wait { inLoop },
        exec {
            // Alliance selection during start (before autonomous begins)
            if (gamepad1.dpad_up) {
                alliance = Alliance.BLUE
                gamepad1.setLedColor(0.0, 0.0, 1.0, com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS)
            } else if (gamepad1.dpad_down) {
                alliance = Alliance.RED
                gamepad1.setLedColor(1.0, 0.0, 0.0, com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS)
            }

            setPathState(0)
        },
        loop({ pathState < 2 }, exec {
            container.follower.update()
            autonomousPathUpdate()
        })
    )

    dropToScheduler()

    driverTeleOp.program
}
