package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import software.amazon.lastmile.kotlin.inject.anvil.ContributesBinding
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn

@Inject
@SingleIn(HardwareScope::class)
class IntakeSubsystem(hardwareMap: HardwareMap) {
    private val motor = hardwareMap.dcMotor.get("intake").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun collect() = exec {
        motor.power = 0.9
    }

    fun eject() = exec {
        motor.power = -0.9
    }

    fun stop() = exec {
        motor.power = 0.0
    }
}
