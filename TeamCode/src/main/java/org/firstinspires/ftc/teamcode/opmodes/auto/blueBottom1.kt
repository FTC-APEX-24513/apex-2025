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
val blueBottom1 = Mercurial.autonomous {

    lateinit var container: HardwareContainer
    lateinit var pathTimer: Timer
    lateinit var actionTimer: Timer

    var pathState = 0
    var actionState = 0

    val startPoseBlueBottom = Pose(56.0, 16.0, Math.toRadians(180.0))
    val blueBottomRowStart = Pose(40.0, 36.0, Math.toRadians(180.0))
    val blueBottomRowEnd = Pose(20.0, 36.0, Math.toRadians(180.0))
    val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    lateinit var blueStartToScore: PathChain
    lateinit var blueScoreToBottom: PathChain
    lateinit var blueBottomIntake: PathChain
    lateinit var blueBottomToScore: PathChain

    fun buildPaths() {
        blueStartToScore = container.follower.pathBuilder()
            .addPath(Path(BezierLine(startPoseBlueBottom, blueScore)))
            .setLinearHeadingInterpolation(startPoseBlueBottom.heading, blueScore.heading)
            .build()

        blueScoreToBottom = container.follower.pathBuilder()
            .addPath(Path(BezierLine(blueScore, blueBottomRowStart)))
            .setLinearHeadingInterpolation(blueScore.heading, blueBottomRowStart.heading)
            .build()

        blueBottomIntake = container.follower.pathBuilder()
            .addPath(Path(BezierLine(blueBottomRowStart, blueBottomRowEnd)))
            .setLinearHeadingInterpolation(blueBottomRowStart.heading, blueBottomRowEnd.heading)
            .build()

        blueBottomToScore = container.follower.pathBuilder()
            .addPath(Path(BezierLine(blueBottomRowEnd, blueScore)))
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
                container.intake.collect()
                container.follower.followPath(blueStartToScore)
                setPathState(1)
            }
            1 -> if (!container.follower.isBusy) {
                container.intake.stop()
                container.limelight.useAprilTagPipeline()
                actionTimer.resetTimer()
                actionState = 0
                setPathState(2)
            }
            2 -> if (!container.follower.isBusy) {
                container.follower.followPath(blueScoreToBottom)
                setPathState(3)
            }
            3 -> if (!container.follower.isBusy) {
                container.intake.collect()
                container.follower.followPath(blueBottomIntake)
                setPathState(4)
            }
            4 -> if (!container.follower.isBusy) {
                container.intake.stop()
                container.follower.followPath(blueBottomToScore)
                actionTimer.resetTimer()
                actionState = 0
                setPathState(5)
            }
            5 -> { }
            6 -> { }
        }
    }

    fun shootingSequence() {
        if (pathState != 5) return

        when (actionState) {
            0 -> if (actionTimer.elapsedTime > 500) { actionTimer.resetTimer(); actionState = 1 }
            1 -> if (actionTimer.elapsedTime > 350) {
                schedule(container.transfer.transfer())
                actionTimer.resetTimer()
                actionState = 2
            }
            2 -> if (actionTimer.elapsedTime > 600) {
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                actionTimer.resetTimer()
                actionState = 3
            }
            3 -> if (actionTimer.elapsedTime > 300) {
                schedule(sequence(container.transfer.reset(), container.spindexer.rotateRight()))
                actionTimer.resetTimer()
                actionState = 4
            }
            4 -> if (actionTimer.elapsedTime > 350) {
                schedule(container.transfer.transfer())
                actionTimer.resetTimer()
                actionState = 5
            }
            5 -> if (actionTimer.elapsedTime > 600) {
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                actionTimer.resetTimer()
                actionState = 6
            }
            6 -> if (actionTimer.elapsedTime > 300) {
                schedule(sequence(container.transfer.reset(), container.spindexer.rotateRight()))
                actionTimer.resetTimer()
                actionState = 7
            }
            7 -> if (actionTimer.elapsedTime > 350) {
                schedule(container.transfer.transfer())
                actionTimer.resetTimer()
                actionState = 8
            }
            8 -> if (actionTimer.elapsedTime > 600) { setPathState(6) }
        }
    }

    sequence(
        exec {
            pathTimer = Timer()
            actionTimer = Timer()

            container = HardwareContainer::class.create(hardwareMap, scheduler).also {
                it.startPeriodic()
            }

            buildPaths()
            container.follower.setStartingPose(startPoseBlueBottom)
        },
        wait { inLoop },
        exec { setPathState(0) },
        loop(exec {
            container.follower.update()
            autonomousPathUpdate()
            shootingSequence()
        })
    )

    dropToScheduler()
}
