package org.firstinspires.ftc.teamcode.opmodes.auto

import com.bylazar.telemetry.PanelsTelemetry
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create

@Suppress("UNUSED")
val bottomShooting = Mercurial.autonomous {
    val telemetry = PanelsTelemetry.telemetry
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    waitForStart()
    container.follower.startTeleopDrive(true)

    schedule(
        sequence(
            container.outtake.setPower(0.95),
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
            exec { container.follower.setTeleOpDrive(0.5, 0.0, 0.0) },
            wait(0.4),
            exec { container.follower.setTeleOpDrive(0.0, 0.0, 0.0) }
        )
    )

    dropToScheduler()
}
