package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import kotlin.math.abs

@Inject
@HardwareScoped
class SpindexerSubsystem(hardwareMap: HardwareMap) : Subsystem() {
    private val servo = hardwareMap.get(CRServo::class.java, "spindexer")

    private val currentPosition: Double
        get() = servo.controller.getServoPosition(servo.portNumber)

    sealed interface State {
        object Idle : State
        data class RotatingLeft(val targetPosition: Double) : State
        data class RotatingRight(val targetPosition: Double) : State
        object ContinuousLeft : State
        object ContinuousRight : State
    }

    var state: State = State.Idle
        private set

    override fun periodic(): Closure = exec {
        when (val s = state) {
            is State.Idle -> servo.power = 0.0
            is State.RotatingLeft -> {
                if (abs(currentPosition - s.targetPosition) > 0.1) {
                    servo.power = -RobotConstants.SPINDEXER_ROTATION_POWER
                } else {
                    state = State.Idle
                }
            }

            is State.RotatingRight -> {
                if (abs(currentPosition - s.targetPosition) > 0.1) {
                    servo.power = RobotConstants.SPINDEXER_ROTATION_POWER
                } else {
                    state = State.Idle
                }
            }
            is State.ContinuousLeft -> servo.power = -RobotConstants.SPINDEXER_ROTATION_POWER
            is State.ContinuousRight -> servo.power = RobotConstants.SPINDEXER_ROTATION_POWER
        }
    }

    // Commands
    fun stop(): Closure = exec { state = State.Idle }

    // Single position rotation (for tap)
    fun rotateLeftOnce(): Closure = exec {
        state = State.RotatingLeft(currentPosition - RobotConstants.SPINDEXER_TICKS_PER_POSITION)
    }

    fun rotateRightOnce(): Closure = exec {
        state = State.RotatingRight(currentPosition + RobotConstants.SPINDEXER_TICKS_PER_POSITION)
    }

    // Continuous rotation (for hold)
    fun continuousLeft(): Closure = exec { state = State.ContinuousLeft }
    fun continuousRight(): Closure = exec { state = State.ContinuousRight }
}
