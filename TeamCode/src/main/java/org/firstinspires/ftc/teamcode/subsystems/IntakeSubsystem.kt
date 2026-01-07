package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareScoped

@Inject
@HardwareScoped
class IntakeSubsystem(hardwareMap: HardwareMap) : Subsystem() {
    private val motor = hardwareMap.dcMotor.get("intake").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
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
            is State.Collecting -> RobotConstants.INTAKE_COLLECT_POWER
            is State.Ejecting -> RobotConstants.INTAKE_EJECT_POWER
        }
    }

    fun collect(): Closure = exec { state = State.Collecting }
    fun eject(): Closure = exec { state = State.Ejecting }
    fun stop(): Closure = exec { state = State.Idle }
}
