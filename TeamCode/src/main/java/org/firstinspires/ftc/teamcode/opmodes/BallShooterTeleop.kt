package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem


@TeleOp
class BallShooterTeleop : OpMode() {
    private lateinit var intakeSubsystem: IntakeSubsystem


    override fun init() {
        gamepad1.setLedColor(195.0, 247.0, 202.0, 10)
        intakeSubsystem = IntakeSubsystem(hardwareMap);
    }

    override fun loop() {
        if (gamepad1.left_trigger > 0.5) {
            intakeSubsystem.collect()
        } else {
            intakeSubsystem.stop()
        }
    }
}