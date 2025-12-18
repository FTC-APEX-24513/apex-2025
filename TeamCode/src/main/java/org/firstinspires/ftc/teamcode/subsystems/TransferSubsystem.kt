package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import software.amazon.lastmile.kotlin.inject.anvil.ContributesBinding
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn

@Inject
@SingleIn(HardwareScope::class)
class TransferSubsystem(hardwareMap: HardwareMap) {
    private val servo = hardwareMap.servo.get("transfer").apply {
        position = DEFAULT_POSITION
    }

    companion object {
        private const val DEFAULT_POSITION = 0.2639
    }

    fun transfer() = exec {
        // TODO: Implement transfer logic
    }

    fun setPosition(position: Double) = exec {
        servo.position = position
    }

    fun reset() = exec {
        servo.position = DEFAULT_POSITION
    }
}
