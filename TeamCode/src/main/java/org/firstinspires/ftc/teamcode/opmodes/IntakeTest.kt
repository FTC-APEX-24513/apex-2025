package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo

@TeleOp(name = "Intake Left Trigger")
class IntakeLeftTrigger : OpMode() {

    private lateinit var intake: CRServo
    private val INTAKE_POWER = 0.95

    override fun init() {
        intake = hardwareMap.get(CRServo::class.java, "intake")
        intake.power = 0.0
    }

    override fun loop() {
        intake.power = if (gamepad1.left_trigger > 0.1) {
            INTAKE_POWER
        } else {
            0.0
        }
    }

    override fun stop() {
        intake.power = 0.0
    }
}