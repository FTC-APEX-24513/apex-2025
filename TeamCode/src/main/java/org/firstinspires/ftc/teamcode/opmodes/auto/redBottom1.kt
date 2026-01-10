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
val redBottom1 = Mercurial.autonomous {

    lateinit var container: HardwareContainer
    lateinit var pathTimer: Timer
    var pathState = 0

    val startPose = Pose(88.0, 16.0, Math.toRadians(180.0))
    val redBottomRowStart = Pose(104.0, 36.0, Math.toRadians(180.0))
    val redBottomRowEnd = Pose(124.0, 36.0, Math.toRadians(180.0))
    val redScore = Pose(96.0, 48.0, Math.toRadians(0.0))

    lateinit var scorePreload: Path
    lateinit var redScoreToBottom: PathChain
    lateinit var redBottomIntake: PathChain
    lateinit var redBottomToScore: PathChain

    fun buildPaths() {
        scorePreload = Path(BezierLine(startPose, redScore)).apply {
            setLinearHeadingInterpolation(startPose.heading, redScore.heading)
        }

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
            pathTimer = Timer()

            container = HardwareContainer::class.create(hardwareMap, scheduler).also {
                it.startPeriodic()
            }

            buildPaths()
            container.follower.setStartingPose(startPose)
        },
        wait { inLoop },
        exec { pathState = 0 },
        loop(
            exec {
                container.follower.update()
                when (pathState) {
                    0 -> {
                        container.follower.followPath(scorePreload)
                        container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                        pathState = 1
                    }
                    1 -> if (!container.follower.isBusy) {
                        container.follower.followPath(redScoreToBottom)
                        pathState = 2
                    }
                    2 -> if (!container.follower.isBusy) {
                        container.intake.collect()
                        container.follower.followPath(redBottomIntake)
                        pathState = 3
                    }
                    3 -> if (!container.follower.isBusy) {
                        container.intake.stop()
                        container.follower.followPath(redBottomToScore)
                        container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                        pathState = 4
                    }
                    4 -> if (!container.follower.isBusy) pathState = 5
                    5 -> { /* Finished */ }
                }
            }
        )
    )

    dropToScheduler()
}
