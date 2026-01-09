package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareScoped

@Inject
@HardwareScoped
class TransferSubsystem(hardwareMap: HardwareMap) : Subsystem() {
    private val servo = hardwareMap.get(Servo::class.java, "transfer")

    sealed interface State {
        object Default : State
        data class AtPosition(val position: Double) : State
    }

    var state: State = State.Default
        private set

    override fun periodic(): Closure = exec {
        servo.position = when (val s = state) {
            is State.Default -> RobotConstants.TRANSFER_DEFAULT_POSITION
            is State.AtPosition -> s.position
        }
    }

    /**
     * Get the current target position of the servo.
     */
    fun getTargetPosition(): Double = when (val s = state) {
        is State.Default -> RobotConstants.TRANSFER_DEFAULT_POSITION
        is State.AtPosition -> s.position
    }

    /**
     * Increment the position by a delta (for tuning).
     */
    fun adjustPosition(delta: Double): Closure = exec {
        val current = getTargetPosition()
        state = State.AtPosition((current + delta).coerceIn(0.0, 1.0))
    }

    // Commands
    fun transfer(): Closure = exec { state = State.AtPosition(RobotConstants.TRANSFER_TRANSFER_POSITION) }
    fun reset(): Closure = exec { state = State.Default }
    fun setPosition(position: Double): Closure = exec { state = State.AtPosition(position.coerceIn(0.0, 1.0)) }

    fun setPositionDirect(position: Double) {
        state = State.AtPosition(position.coerceIn(0.0, 1.0))
    }
}
