package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial

@Suppress("UNUSED")
val hoodTest = Mercurial.teleop {
    val hood0 = hardwareMap.servo.get("hood0").apply {
        this.direction = Servo.Direction.FORWARD
    }
    val hood1 = hardwareMap.servo.get("hood1").apply {
        this.direction = Servo.Direction.REVERSE
    }

    waitForStart()

    bindSpawn(risingEdge { gamepad1.dpad_up }, exec { hood0.position = 1.0; hood1.position = 1.0 })
    bindSpawn(risingEdge { gamepad1.dpad_down }, exec { hood0.position = 0.0; hood1.position = 0.0 })

    dropToScheduler()
}