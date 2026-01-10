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
val basicAnywhere = Mercurial.autonomous {
    val telemetryA = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    var pathState = 0

    waitForStart()
    container.follower.startTeleopDrive(true)

    schedule(
        parallel(
            loop({ true }, exec { container.follower.update() }),
            sequence(
                exec {
                    container.follower.setTeleOpDrive(0.0, 0.5, 0.0)
                },
                wait(0.4),
                exec {
                    container.follower.setTeleOpDrive(0.0, 0.0, 0.0)
                }
            )
        )
    )

    dropToScheduler()
}
