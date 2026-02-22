package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.hardware.DcMotor
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial

@Suppress("UNUSED")
val dtTest = Mercurial.teleop {
    val frontRight = hardwareMap.dcMotor["frontRight"].apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val backRight = hardwareMap.dcMotor["backRight"].apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val frontLeft = hardwareMap.dcMotor["frontLeft"].apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val backLeft = hardwareMap.dcMotor["backLeft"].apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    waitForStart()

    schedule(
        loop({ inLoop }, exec {

            if (frontRight.power != 0.0) telemetry.addData("frontRight", frontRight.power);
            if (backRight.power != 0.0) telemetry.addData("backRight", backRight.power);
            if (frontLeft.power != 0.0) telemetry.addData("frontLeft", frontLeft.power);
            if (backLeft.power != 0.0) telemetry.addData("backLeft", backLeft.power);

            telemetry.update()
        })
    )

    bindSpawn(risingEdge { gamepad1.dpad_up }, exec { frontRight.power = 1.0 })
    bindSpawn(risingEdge { !gamepad1.dpad_up }, exec { frontRight.power = 0.0 })
    bindSpawn(risingEdge { gamepad1.dpad_left }, exec { backRight.power = 1.0 })
    bindSpawn(risingEdge { !gamepad1.dpad_left }, exec { backRight.power = 0.0 })
    bindSpawn(risingEdge { gamepad1.dpad_down }, exec { frontLeft.power = 1.0 })
    bindSpawn(risingEdge { !gamepad1.dpad_down }, exec { frontLeft.power = 0.0 })
    bindSpawn(risingEdge { gamepad1.dpad_right }, exec { backLeft.power = 1.0 })
    bindSpawn(risingEdge { !gamepad1.dpad_right }, exec { backLeft.power = 0.0 })

    dropToScheduler()
}