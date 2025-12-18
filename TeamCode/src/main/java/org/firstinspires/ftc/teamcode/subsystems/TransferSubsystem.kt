package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.RobotScope
import software.amazon.lastmile.kotlin.inject.anvil.ContributesBinding
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn

@Inject
@SingleIn(RobotScope::class)
@ContributesBinding(RobotScope::class, multibinding = true)
class TransferSubsystem : Subsystem {
    private lateinit var servo: Servo

    companion object {
        private const val DEFAULT_POSITION = 0.2639
    }

    override fun init(hardwareMap: HardwareMap) {
        servo = hardwareMap.servo.get("transfer").apply {
            position = DEFAULT_POSITION
        }
    }

    fun transfer() {
        // TODO: Implement transfer logic
    }

    fun setPosition(position: Double) {
        servo.position = position
    }

    fun reset() {
        servo.position = DEFAULT_POSITION
    }

    override fun periodic() {
        // Optional: Add state machine logic
    }
}
