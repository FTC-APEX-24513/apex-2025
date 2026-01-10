package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

@Suppress("UNUSED")
val redBottom0 = Mercurial.autonomous {

    lateinit var container: HardwareContainer
    lateinit var pathTimer: Timer
    var pathState = 0

    val startPose = Pose(88.0, 16.0, Math.toRadians(180.0))
    val redScore = Pose(96.0, 48.0, Math.toRadians(0.0))

    lateinit var redStartToScore: PathChain

    fun buildPaths() {
        redStartToScore = container.follower.pathBuilder()
            .addPath(BezierLine(startPose, redScore))
            .setLinearHeadingInterpolation(startPose.heading, redScore.heading)
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
                        container.follower.followPath(redStartToScore)
                        container.outtake.setState(OuttakeSubsystem.State.ManualPower(1.0))
                        pathState = 1
                    }
                    1 -> if (!container.follower.isBusy) pathState = 2
                    2 -> { /* Finished */ }
                }
            }
        )
    )

    dropToScheduler()
}
