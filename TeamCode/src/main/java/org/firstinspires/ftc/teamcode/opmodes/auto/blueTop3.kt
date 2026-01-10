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
val blueTop3 = Mercurial.autonomous {

    lateinit var container: HardwareContainer
    lateinit var pathTimer: Timer

    var pathState = 0

    lateinit var scorePreload: Path
    lateinit var blueScoreToTopStart: PathChain
    lateinit var blueTopStartToTopEnd: PathChain
    lateinit var blueTopEndToScore: PathChain
    lateinit var blueScoreToMiddleStart: PathChain
    lateinit var blueMiddleStartToMiddleEnd: PathChain
    lateinit var blueMiddleEndToScore: PathChain
    lateinit var blueScoreToBottomStart: PathChain
    lateinit var blueBottomStartToBottomEnd: PathChain
    lateinit var blueBottomEndToScore: PathChain

    val startPoseBlueTop = Pose(56.0, 88.0, Math.toRadians(180.0))
    val blueTopRowStart = Pose(40.0, 84.0, Math.toRadians(180.0))
    val blueTopRowEnd = Pose(20.0, 84.0, Math.toRadians(180.0))
    val blueMiddleRowStart = Pose(40.0, 60.0, Math.toRadians(180.0))
    val blueMiddleRowEnd = Pose(20.0, 60.0, Math.toRadians(180.0))
    val blueBottomRowStart = Pose(40.0, 36.0, Math.toRadians(180.0))
    val blueBottomRowEnd = Pose(20.0, 36.0, Math.toRadians(180.0))
    val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    fun buildPaths() {
        scorePreload = Path(BezierLine(startPoseBlueTop, blueScore))
        scorePreload.setLinearHeadingInterpolation(startPoseBlueTop.heading, blueScore.heading)

        blueScoreToTopStart = container.follower.pathBuilder()
            .addPath(BezierLine(blueScore, blueTopRowStart))
            .setLinearHeadingInterpolation(blueScore.heading, blueTopRowStart.heading)
            .build()

        blueTopStartToTopEnd = container.follower.pathBuilder()
            .addPath(BezierLine(blueTopRowStart, blueTopRowEnd))
            .setLinearHeadingInterpolation(blueTopRowStart.heading, blueTopRowEnd.heading)
            .build()

        blueTopEndToScore = container.follower.pathBuilder()
            .addPath(BezierLine(blueTopRowEnd, blueScore))
            .setLinearHeadingInterpolation(blueTopRowEnd.heading, blueScore.heading)
            .build()

        blueScoreToMiddleStart = container.follower.pathBuilder()
            .addPath(BezierLine(blueScore, blueMiddleRowStart))
            .setLinearHeadingInterpolation(blueScore.heading, blueMiddleRowStart.heading)
            .build()

        blueMiddleStartToMiddleEnd = container.follower.pathBuilder()
            .addPath(BezierLine(blueMiddleRowStart, blueMiddleRowEnd))
            .setLinearHeadingInterpolation(blueMiddleRowStart.heading, blueMiddleRowEnd.heading)
            .build()

        blueMiddleEndToScore = container.follower.pathBuilder()
            .addPath(BezierLine(blueMiddleRowEnd, blueScore))
            .setLinearHeadingInterpolation(blueMiddleRowEnd.heading, blueScore.heading)
            .build()

        blueScoreToBottomStart = container.follower.pathBuilder()
            .addPath(BezierLine(blueScore, blueBottomRowStart))
            .setLinearHeadingInterpolation(blueScore.heading, blueBottomRowStart.heading)
            .build()

        blueBottomStartToBottomEnd = container.follower.pathBuilder()
            .addPath(BezierLine(blueBottomRowStart, blueBottomRowEnd))
            .setLinearHeadingInterpolation(blueBottomRowStart.heading, blueBottomRowEnd.heading)
            .build()

        blueBottomEndToScore = container.follower.pathBuilder()
            .addPath(BezierLine(blueBottomRowEnd, blueScore))
            .setLinearHeadingInterpolation(blueBottomRowEnd.heading, blueScore.heading)
            .build()
    }

    fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                container.follower.followPath(scorePreload)
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                pathState = 1
            }
            1 -> if (!container.follower.isBusy) {
                container.follower.followPath(blueScoreToTopStart)
                pathState = 2
            }
            2 -> if (!container.follower.isBusy) {
                container.intake.collect()
                container.follower.followPath(blueTopStartToTopEnd)
                pathState = 3
            }
            3 -> if (!container.follower.isBusy) {
                container.intake.stop()
                container.follower.followPath(blueTopEndToScore)
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                pathState = 4
            }
            4 -> if (!container.follower.isBusy) {
                container.follower.followPath(blueScoreToMiddleStart)
                pathState = 5
            }
            5 -> if (!container.follower.isBusy) {
                container.intake.collect()
                container.follower.followPath(blueMiddleStartToMiddleEnd)
                pathState = 6
            }
            6 -> if (!container.follower.isBusy) {
                container.intake.stop()
                container.follower.followPath(blueMiddleEndToScore)
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                pathState = 7
            }
            7 -> if (!container.follower.isBusy) {
                container.follower.followPath(blueScoreToBottomStart)
                pathState = 8
            }
            8 -> if (!container.follower.isBusy) {
                container.intake.collect()
                container.follower.followPath(blueBottomStartToBottomEnd)
                pathState = 9
            }
            9 -> if (!container.follower.isBusy) {
                container.intake.stop()
                container.follower.followPath(blueBottomEndToScore)
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                pathState = 10
            }
            10 -> if (!container.follower.isBusy) {
                pathState = 11
            }
            11 -> { }
        }
    }

    sequence(
        exec {
            pathTimer = Timer()

            container = HardwareContainer::class.create(hardwareMap, scheduler).also {
                it.startPeriodic()
            }

            buildPaths()
            container.follower.setStartingPose(startPoseBlueTop)
        },
        wait { inLoop },
        exec { setPathState(0) },
        loop(exec {
            container.follower.update()
            autonomousPathUpdate()
        })
    )

    dropToScheduler()
}
