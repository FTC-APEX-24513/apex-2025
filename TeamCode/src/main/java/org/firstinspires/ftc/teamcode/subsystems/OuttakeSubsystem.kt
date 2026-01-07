package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import kotlin.math.abs

@Inject
@HardwareScoped
class OuttakeSubsystem(hardwareMap: HardwareMap) : Subsystem() {
    private val motor = hardwareMap.get(DcMotorEx::class.java, "flywheel").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    sealed interface State {
        object Off : State
        data class SpinningUp(val targetRPM: Double) : State
        data class Ready(val targetRPM: Double) : State
        object Launching : State
        data class ManualPower(val power: Double) : State
    }

    var state: State = State.Off
        private set

    fun setState(newState: State) {
        state = newState
    }

    override fun periodic(): Closure = exec {
        when (val s = state) {
            is State.Off -> motor.power = 0.0
            is State.SpinningUp -> {
                motor.power = calculatePowerForRPM(s.targetRPM, getCurrentRPM())
                if (isAtTargetSpeed(s.targetRPM)) {
                    state = State.Ready(s.targetRPM)
                }
            }
            is State.Ready -> {
                motor.power = calculatePowerForRPM(s.targetRPM, getCurrentRPM())
            }
            is State.Launching -> {
                motor.power = 1.0
            }
            is State.ManualPower -> {
                motor.power = s.power
            }
        }
    }

    fun getCurrentRPM(): Double {
        val ticksPerSecond = motor.velocity
        val revolutionsPerSecond = ticksPerSecond / RobotConstants.OUTTAKE_TICKS_PER_REVOLUTION
        return revolutionsPerSecond * 60.0
    }

    fun isAtTargetSpeed(targetRPM: Double, tolerance: Double = 0.05): Boolean {
        val currentRPM = getCurrentRPM()
        val error = abs(targetRPM - currentRPM) / targetRPM.coerceAtLeast(1.0)
        return error < tolerance
    }

    private fun calculatePowerForRPM(targetRPM: Double, currentRPM: Double): Double {
        val error = targetRPM - currentRPM
        val power = RobotConstants.OUTTAKE_RPM_PROPORTIONAL_GAIN * error
        return power.coerceIn(0.0, 1.0)
    }

    // Commands
    fun spinUp(): Closure = exec { state = State.SpinningUp(RobotConstants.OUTTAKE_DEFAULT_SPINUP_RPM) }
    fun spinToRPM(targetRPM: Double): Closure = exec { state = State.SpinningUp(targetRPM) }
    fun setPower(power: Double): Closure = exec { state = State.ManualPower(power) }
    fun stop(): Closure = exec { state = State.Off }

    fun launch(): Closure = sequence(
        exec { state = State.Launching },
        wait(RobotConstants.OUTTAKE_LAUNCH_DURATION),
        exec { state = State.Off }
    )
}
