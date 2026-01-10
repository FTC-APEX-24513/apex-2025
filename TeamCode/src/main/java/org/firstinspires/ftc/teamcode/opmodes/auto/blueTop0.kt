package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
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
val blueTop0 = Mercurial.autonomous {

    lateinit var container: HardwareContainer
    lateinit var pathTimer: Timer

    var pathState = 0

    lateinit var scorePreload: Path

    val startPoseBlueTop = Pose(56.0, 88.0, Math.toRadians(180.0))
    val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    fun buildPaths() {
        scorePreload = Path(BezierLine(startPoseBlueTop, blueScore))
        scorePreload.setLinearHeadingInterpolation(
            startPoseBlueTop.heading,
            blueScore.heading
        )
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
                setPathState(1)
            }
            1 -> if (!container.follower.isBusy) {
                pathState = 2
            }
            2 -> { }
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
