package org.firstinspires.ftc.teamcode.opmodes

import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create

@Suppress("UNUSED")
val transferTest = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val transfer = container.transfer

    waitForStart()

    bindSpawn(risingEdge { gamepad1.square }, transfer.trigger(0))
    bindSpawn(risingEdge { gamepad1.cross }, transfer.trigger(1))
    bindSpawn(risingEdge { gamepad1.triangle }, transfer.trigger(2))

    bindSpawn(
        risingEdge { gamepad1.circle },
        transfer.triggerSequence(arrayOf(0, 1, 2))
    )

    dropToScheduler()
}