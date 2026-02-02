package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.DcMotor
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create

/**
 * Main Driver TeleOp mode.
 */
@Suppress("UNUSED")
val encoderTest = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val analogInput = hardwareMap.analogInput.get("spindexerEncoder").apply {
    }
    val revEncoder = hardwareMap.dcMotor.get("spindexerREVEncoder").apply {
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    waitForStart()

    bindSpawn(risingEdge { gamepad1.right_bumper }, container.spindexer.advance())
    bindSpawn(risingEdge { gamepad1.left_bumper }, container.spindexer.reverse())

    schedule(
        loop({ inLoop }, exec {
            val voltage = analogInput.voltage
            val revPosition = revEncoder.currentPosition
            telemetry.addData("Analog Voltage", voltage)
            telemetry.addData("Analog Encoder Position", (voltage / 3.3) * 8192)
            telemetry.addData("REV Encoder Position", revPosition)
            telemetry.addData("Spindexer State", container.spindexer.state)
            telemetry.addData("Spindexer Slot", container.spindexer.getCurrentSlot())
            telemetry.update()
        })
    )

    dropToScheduler()
}