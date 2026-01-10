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
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

/**
 * Red Top Pipeline Autonomous -> TeleOp
 *
 * Starting position: Red alliance, top position (near observation zone)
 * Autonomous: Simple preload score, then transitions to driver control
 */
@Suppress("UNUSED")
val aRedTopPipeline = Mercurial.pipelineAutonomous {

    lateinit var container: HardwareContainer
    lateinit var pathTimer: Timer
    val telemetryA = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    var pathState = 0

    // Red Top starting position (near observation zone, mirrored from blue)
    val startPoseRedTop = Pose(40.0, 84.0, Math.toRadians(0.0))
    val redScore = Pose(48.0, 72.0, Math.toRadians(0.0))

    lateinit var redTopToScore: PathChain

    fun buildPaths() {
        redTopToScore = container.follower.pathBuilder()
            .addPath(Path(BezierLine(startPoseRedTop, redScore)))
            .setLinearHeadingInterpolation(
                startPoseRedTop.heading,
                redScore.heading
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
                // Drive to scoring position
                container.follower.followPath(redTopToScore)
                setPathState(1)
            }
            1 -> {
                if (!container.follower.isBusy) {
                    // Lock flywheel to distance and spin up
                    val distance = container.getGoalDistance(Alliance.RED)
                    if (distance != null) {
                        container.outtake.lockToDistanceDirect(distance)
                    } else {
                        container.outtake.spinToRPMDirect(OuttakeSubsystem.DEFAULT_SHOOTING_RPM)
                    }
                    setPathState(2)
                }
            }
            2 -> {
                // Wait for flywheel to reach target speed
                if (container.outtake.isReady()) {
                    setPathState(3)
                }
            }
            3 -> {
                // Done - transition to teleop
                container.limelight.useAprilTagPipeline()
                setPathState(4)
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
            container.follower.setStartingPose(startPoseRedTop)

            // Set LED to red
            gamepad1.setLedColor(1.0, 0.0, 0.0, com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS)
        },
        wait { inLoop },
        exec {
            setPathState(0)
        },
        loop({ pathState < 4 }, exec {
            container.follower.update()
            autonomousPathUpdate()

            // Telemetry
            telemetryA.addLine("=== RED TOP AUTONOMOUS ===")
            telemetryA.addData("State", pathState)
            telemetryA.addData("Flywheel RPM", container.outtake.getCurrentRPM().toInt())
            telemetryA.addData("Flywheel Ready", container.outtake.isReady())
            telemetryA.update()
        })
    )

    dropToScheduler()

    driverTeleOp.program
}
