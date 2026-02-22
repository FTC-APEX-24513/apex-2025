package org.firstinspires.ftc.teamcode.opmodes.test

import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem

@Suppress("UNUSED")
val intakeTest = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val intake = container.intake
    var isOn = false

    waitForStart()

    bindSpawn(risingEdge { gamepad1.right_bumper }, intake.collect())
    bindSpawn(risingEdge { gamepad1.left_bumper }, intake.stop())

    bindSpawn(risingEdge { gamepad1.dpad_up }, exec { IntakeSubsystem.COLLECT_POWER = (IntakeSubsystem.COLLECT_POWER + 0.1).coerceIn(-1.0, 1.0) })
    bindSpawn(risingEdge { gamepad1.dpad_down }, exec { IntakeSubsystem.COLLECT_POWER = (IntakeSubsystem.COLLECT_POWER - 0.1).coerceIn(-1.0, 1.0) })

    schedule(
        loop({ inLoop }, exec {
            telemetry.addData("Intake power", IntakeSubsystem.COLLECT_POWER)
            telemetry.update()
        })
    )

    dropToScheduler()
}