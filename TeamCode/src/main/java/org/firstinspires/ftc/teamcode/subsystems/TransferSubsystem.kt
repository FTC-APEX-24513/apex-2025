package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScoped

@Config
@Inject
@HardwareScoped
class TransferSubsystem(hardwareMap: HardwareMap) : Subsystem() {
    private val servo = hardwareMap.get(Servo::class.java, "transfer")

    companion object {
        @JvmField var DEFAULT_POSITION = 0.2639
        @JvmField var TRANSFER_POSITION = -1.0
    }

    sealed interface State {
        object Default : State
        data class AtPosition(val position: Double) : State
    }

    var state: State = State.Default
        private set

    override fun periodic(): Closure = exec {
        servo.position = when (val s = state) {
            is State.Default -> DEFAULT_POSITION
            is State.AtPosition -> s.position
        }
    }

    fun getTargetPosition(): Double = when (val s = state) {
        is State.Default -> DEFAULT_POSITION
        is State.AtPosition -> s.position
    }

    fun adjustPosition(delta: Double): Closure = exec {
        val current = getTargetPosition()
        state = State.AtPosition((current + delta).coerceIn(0.0, 1.0))
    }

    // Commands
    fun transfer(): Closure = exec { state = State.AtPosition(TRANSFER_POSITION) }
    fun reset(): Closure = exec { state = State.Default }
    fun setPosition(position: Double): Closure = exec { state = State.AtPosition(position.coerceIn(0.0, 1.0)) }

    fun setPositionDirect(position: Double) {
        state = State.AtPosition(position.coerceIn(0.0, 1.0))
    }
}
