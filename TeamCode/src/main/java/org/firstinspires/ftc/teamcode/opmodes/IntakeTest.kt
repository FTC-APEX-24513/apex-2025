package org.firstinspires.ftc.teamcode.opmodes

import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create

@Suppress("UNUSED")
val intakeTest = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val intake = container.intake

    waitForStart()

    bindSpawn(risingEdge { gamepad1.right_bumper }, container.intake.collect())
    bindSpawn(risingEdge { !gamepad1.right_bumper }, container.intake.stop())

    dropToScheduler()
}