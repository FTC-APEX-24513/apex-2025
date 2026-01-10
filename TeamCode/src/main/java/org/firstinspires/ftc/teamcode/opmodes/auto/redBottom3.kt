package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

@Suppress("UNUSED")
val redBottom3 = Mercurial.autonomous {

    lateinit var container: HardwareContainer
    val pathTimer = Timer()
    var pathState = 0

    val startPoseRedBottom = Pose(88.0, 16.0, Math.toRadians(180.0))
    val redTopRowStart = Pose(104.0, 84.0, Math.toRadians(180.0))
    val redTopRowEnd = Pose(124.0, 84.0, Math.toRadians(180.0))
    val redMiddleRowStart = Pose(104.0, 60.0, Math.toRadians(180.0))
    val redMiddleRowEnd = Pose(124.0, 60.0, Math.toRadians(180.0))
    val redBottomRowStart = Pose(104.0, 36.0, Math.toRadians(180.0))
    val redBottomRowEnd = Pose(124.0, 36.0, Math.toRadians(180.0))
    val redScore = Pose(112.0, 48.0, Math.toRadians(180.0))

    lateinit var redStartToScore: Path
    lateinit var redScoreToTop: PathChain
    lateinit var redTopIntake: PathChain
    lateinit var redTopToScore: PathChain
    lateinit var redScoreToMiddle: PathChain
    lateinit var redMiddleIntake: PathChain
    lateinit var redMiddleToScore: PathChain
    lateinit var redScoreToBottom: PathChain
    lateinit var redBottomIntake: PathChain
    lateinit var redBottomToScore: PathChain

    fun buildPaths() {
        redStartToScore = Path(BezierLine(startPoseRedBottom, redScore))
        redStartToScore.setLinearHeadingInterpolation(startPoseRedBottom.heading, redScore.heading)

        redScoreToTop = container.follower.pathBuilder()
            .addPath(BezierLine(redScore, redTopRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redTopRowStart.heading)
            .build()

        redTopIntake = container.follower.pathBuilder()
            .addPath(BezierLine(redTopRowStart, redTopRowEnd))
            .setLinearHeadingInterpolation(redTopRowStart.heading, redTopRowEnd.heading)
            .build()

        redTopToScore = container.follower.pathBuilder()
            .addPath(BezierLine(redTopRowEnd, redScore))
            .setLinearHeadingInterpolation(redTopRowEnd.heading, redScore.heading)
            .build()

        redScoreToMiddle = container.follower.pathBuilder()
            .addPath(BezierLine(redScore, redMiddleRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redMiddleRowStart.heading)
            .build()

        redMiddleIntake = container.follower.pathBuilder()
            .addPath(BezierLine(redMiddleRowStart, redMiddleRowEnd))
            .setLinearHeadingInterpolation(redMiddleRowStart.heading, redMiddleRowEnd.heading)
            .build()

        redMiddleToScore = container.follower.pathBuilder()
            .addPath(BezierLine(redMiddleRowEnd, redScore))
            .setLinearHeadingInterpolation(redMiddleRowEnd.heading, redScore.heading)
            .build()

        redScoreToBottom = container.follower.pathBuilder()
            .addPath(BezierLine(redScore, redBottomRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redBottomRowStart.heading)
            .build()

        redBottomIntake = container.follower.pathBuilder()
            .addPath(BezierLine(redBottomRowStart, redBottomRowEnd))
            .setLinearHeadingInterpolation(redBottomRowStart.heading, redBottomRowEnd.heading)
            .build()

        redBottomToScore = container.follower.pathBuilder()
            .addPath(BezierLine(redBottomRowEnd, redScore))
            .setLinearHeadingInterpolation(redBottomRowEnd.heading, redScore.heading)
            .build()
    }

    sequence(
        exec {
            container = HardwareContainer::class.create(hardwareMap, scheduler).also {
                it.startPeriodic()
            }

            buildPaths()
            container.follower.setStartingPose(startPoseRedBottom)
        },
        wait { inLoop },
        exec { pathState = 0 },
        loop(
            exec {
                container.follower.update()
                when (pathState) {
                    0 -> {
                        container.follower.followPath(redStartToScore)
                        container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                        pathState = 1
                    }
                    1 -> if (!container.follower.isBusy) {
                        container.follower.followPath(redScoreToTop)
                        pathState = 2
                    }
                    2 -> if (!container.follower.isBusy) {
                        container.intake.collect()
                        container.follower.followPath(redTopIntake)
                        pathState = 3
                    }
                    3 -> if (!container.follower.isBusy) {
                        container.intake.stop()
                        container.follower.followPath(redTopToScore)
                        container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                        pathState = 4
                    }
                    4 -> if (!container.follower.isBusy) {
                        container.follower.followPath(redScoreToMiddle)
                        pathState = 5
                    }
                    5 -> if (!container.follower.isBusy) {
                        container.intake.collect()
                        container.follower.followPath(redMiddleIntake)
                        pathState = 6
                    }
                    6 -> if (!container.follower.isBusy) {
                        container.intake.stop()
                        container.follower.followPath(redMiddleToScore)
                        container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                        pathState = 7
                    }
                    7 -> if (!container.follower.isBusy) {
                        container.follower.followPath(redScoreToBottom)
                        pathState = 8
                    }
                    8 -> if (!container.follower.isBusy) {
                        container.intake.collect()
                        container.follower.followPath(redBottomIntake)
                        pathState = 9
                    }
                    9 -> if (!container.follower.isBusy) {
                        container.intake.stop()
                        container.follower.followPath(redBottomToScore)
                        container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                        pathState = 10
                    }
                    10 -> if (!container.follower.isBusy) pathState = 11
                    11 -> { /* Finished */ }
                }
            }
        )
    )

    dropToScheduler()
}
