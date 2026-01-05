package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial

@Suppress("UNUSED")
val myFirstMercurialTeleOp = Mercurial.teleop {
    val fl = hardwareMap.get(DcMotorEx::class.java, "leftFront").apply {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val bl = hardwareMap.get(DcMotorEx::class.java, "leftRear").apply {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val br = hardwareMap.get(DcMotorEx::class.java, "rightRear").apply {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val fr = hardwareMap.get(DcMotorEx::class.java, "rightFront").apply {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    var throttle = 1.0

    // POV drive
    schedule(
        sequence(
            // wait can also take a boolean supplier,
            // we'll start this process now,
            // but it will wait until we press play to actually start running
            wait { inLoop },
            loop(exec {
                val drive = -gamepad1.left_stick_y.toDouble()
                val turn = gamepad1.right_stick_x.toDouble()

                // a simple POV drive
                fl.power = (drive + turn) * throttle
                bl.power = (drive + turn) * throttle
                br.power = (drive - turn) * throttle
                fr.power = (drive - turn) * throttle
            })
        )
    )

    // throttle controls
    bindSpawn(
        risingEdge { gamepad1.right_bumper },
        exec { throttle = 0.5 }
    )

    bindSpawn(
        // inverting the condition will convert our rising edge detector to a falling edge detector!
        risingEdge { !gamepad1.right_bumper },
        exec { throttle = 1.0 }
    )

    dropToScheduler()
}