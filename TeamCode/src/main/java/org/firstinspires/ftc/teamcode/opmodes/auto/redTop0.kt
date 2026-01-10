package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.util.Timer
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.parallel
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

@Suppress("UNUSED")
val aRedTop0 = Mercurial.autonomous {
    val telemetryA = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    var pathState = 0

    val startPoseBlueTop = Pose(20.0, 122.0, Math.toRadians(145.0))
    val blueScore = Pose(72.0, 72.0, Math.toRadians(145.0))
    var scorePreload = Path(BezierLine(startPoseBlueTop, blueScore))
    scorePreload.setLinearHeadingInterpolation(
        startPoseBlueTop.heading, blueScore.heading
    )
    container.follower.setStartingPose(startPoseBlueTop)

    waitForStart()
    container.follower.startTeleopDrive(true)

    schedule(
        parallel(
            loop({ true }, exec { container.follower.update() }),
            sequence(
                exec {
                    container.follower.setTeleOpDrive(-1.0, 0.0, 0.0)
                    container.outtake.spinToRPMDirect(4100.0)
                },
                wait(.95),
                exec {
                    container.follower.setTeleOpDrive(0.0, 0.0, 0.0)
                },
                wait(3.0),
                container.transfer.transfer(),
                wait(2.0),
                container.transfer.reset(),
                wait(.75),
                container.spindexer.rotateLeft(),
                wait(.9),
                container.spindexer.rotateLeft(),
                wait(.9),
                container.transfer.transfer(),
                wait(2.0),
                container.transfer.reset(),
                container.outtake.stop(),
                exec {
                    container.follower.setTeleOpDrive(0.0, -0.5, 0.0)
                },
                wait(0.55),
                exec {
                    container.follower.setTeleOpDrive(0.0, 0.0, 0.0)
                }
            )
        )
    )

    dropToScheduler()
}
