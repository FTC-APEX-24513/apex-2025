package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScoped

@Config
@Inject
@HardwareScoped
class IntakeSubsystem(hardwareMap: HardwareMap) : Subsystem() {
    private val motor = hardwareMap.dcMotor.get("intake").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    companion object {
        @JvmField var COLLECT_POWER = 0.9
        @JvmField var EJECT_POWER = -0.9
        @JvmField var TRIGGER_THRESHOLD = 0.01
    }

    sealed interface State {
        object Idle : State
        object Collecting : State
        object Ejecting : State
    }

    var state: State = State.Idle
        private set

    override fun periodic(): Closure = exec {
        motor.power = when (state) {
            is State.Idle -> 0.0
            is State.Collecting -> COLLECT_POWER
            is State.Ejecting -> EJECT_POWER
        }
    }

    fun collect(): Closure = exec { state = State.Collecting }
    fun eject(): Closure = exec { state = State.Ejecting }
    fun stop(): Closure = exec { state = State.Idle }
}
