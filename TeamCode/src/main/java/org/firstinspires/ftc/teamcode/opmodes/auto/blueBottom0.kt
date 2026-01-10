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
val blueBottom0 = Mercurial.autonomous {

    lateinit var container: HardwareContainer
    lateinit var pathTimer: Timer
    lateinit var actionTimer: Timer

    var pathState = 0
    var actionState = 0

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

    fun shootingSequence() {
        when (actionState) {
            0 -> if (actionTimer.elapsedTimeSeconds > 0.5) { actionTimer.resetTimer(); actionState = 1 }
            1 -> if (actionTimer.elapsedTimeSeconds > 0.35) {
                schedule(container.transfer.transfer())
                actionTimer.resetTimer()
                actionState = 2
            }
            2 -> if (actionTimer.elapsedTimeSeconds > 0.6) {
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                actionTimer.resetTimer()
                actionState = 3
            }
            3 -> if (actionTimer.elapsedTimeSeconds > 0.3) {
                schedule(sequence(container.transfer.reset(), container.spindexer.rotateRight()))
                actionTimer.resetTimer()
                actionState = 4
            }
            4 -> if (actionTimer.elapsedTimeSeconds > 0.35) {
                schedule(container.transfer.transfer())
                actionTimer.resetTimer()
                actionState = 5
            }
            5 -> if (actionTimer.elapsedTimeSeconds > 0.6) {
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                actionTimer.resetTimer()
                actionState = 6
            }
            6 -> if (actionTimer.elapsedTimeSeconds > 0.3) {
                schedule(sequence(container.transfer.reset(), container.spindexer.rotateRight()))
                actionTimer.resetTimer()
                actionState = 7
            }
            7 -> if (actionTimer.elapsedTimeSeconds > 0.35) {
                schedule(container.transfer.transfer())
                actionTimer.resetTimer()
                actionState = 8
            }
            8 -> if (actionTimer.elapsedTimeSeconds > 0.6) {
                container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                setPathState(3)
            }
        }
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
            2 -> shootingSequence()
            3 -> { }
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
        exec {
            setPathState(0)
        },
        loop(exec {
            container.follower.update()
            autonomousPathUpdate()
        })
    )

    dropToScheduler()
}
