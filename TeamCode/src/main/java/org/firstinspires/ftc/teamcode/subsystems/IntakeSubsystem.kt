package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.RobotScope
import software.amazon.lastmile.kotlin.inject.anvil.ContributesBinding
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn

@Inject
@SingleIn(RobotScope::class)
@ContributesBinding(RobotScope::class, multibinding = true)
class IntakeSubsystem : Subsystem {
    private lateinit var motor: DcMotor

    override fun init(hardwareMap: HardwareMap) {
        motor = hardwareMap.dcMotor.get("intake").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    fun collect() {
        motor.power = 0.9
    }

    fun eject() {
        motor.power = -0.9
    }

    fun stop() {
        motor.power = 0.0
    }

    override fun periodic() {
        // Optional: Add telemetry or state machine logic
    }
}
