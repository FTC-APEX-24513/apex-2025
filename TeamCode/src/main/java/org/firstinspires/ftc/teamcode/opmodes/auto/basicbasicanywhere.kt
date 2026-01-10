package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Autonomous(name = "Strafe Right Test", group = "Test")
class StrafeRightTest : LinearOpMode() {

    override fun runOpMode() {
        val leftFront = hardwareMap.get(DcMotor::class.java, "leftFront")
        val leftRear = hardwareMap.get(DcMotor::class.java, "leftRear")
        val rightFront = hardwareMap.get(DcMotor::class.java, "rightFront")
        val rightRear = hardwareMap.get(DcMotor::class.java, "rightRear")

        // Motor directions — EXACTLY as provided
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftRear.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.REVERSE
        rightRear.direction = DcMotorSimple.Direction.FORWARD

        waitForStart()
        if (isStopRequested) return

        // Strafe right (mecanum)
        leftFront.power = 0.5
        leftRear.power = -0.5
        rightFront.power = -0.5
        rightRear.power = 0.5

        sleep(200) // 0.2 seconds

        // Stop
        leftFront.power = 0.0
        leftRear.power = 0.0
        rightFront.power = 0.0
        rightRear.power = 0.0
    }
}