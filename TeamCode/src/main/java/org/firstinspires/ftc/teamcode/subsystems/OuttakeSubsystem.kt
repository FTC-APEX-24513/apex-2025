package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScope
import org.firstinspires.ftc.teamcode.util.VoltageCompensation
import kotlin.math.abs

@Configurable
@Inject
@HardwareScope
class OuttakeSubsystem(hardwareMap: HardwareMap, private val voltageCompensation: VoltageCompensation) : Subsystem() {
    private val motor = hardwareMap.get(DcMotorEx::class.java, "flywheel").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    companion object {
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
            is State.Off -> {
                motor.power = 0.0
            }

            is State.SpinningUp -> {
                val rawPower = calculatePowerForRPM(s.targetRPM, getCurrentRPM())
                motor.power = voltageCompensation.compensate(rawPower)
                if (isAtTargetSpeed(s.targetRPM)) state = State.Ready(s.targetRPM)

            }

            is State.Ready -> {
                val rawPower = calculatePowerForRPM(s.targetRPM, getCurrentRPM())
                motor.power = voltageCompensation.compensate(rawPower)
            }

            is State.Launching -> {
                motor.power = voltageCompensation.compensate(1.0)
            }

            is State.ManualPower -> {
                motor.power = voltageCompensation.compensate(s.power)
            }
        }
    }

    fun getCurrentRPM(): Double {
        val ticksPerSecond = motor.velocity
        val revolutionsPerSecond = ticksPerSecond / 1000
        return revolutionsPerSecond * 60.0
    }

    fun isAtTargetSpeed(targetRPM: Double, tolerance: Double = 0.05): Boolean {
        val currentRPM = getCurrentRPM()
        val error = abs(targetRPM - currentRPM) / targetRPM.coerceAtLeast(1.0)
        return error < tolerance
    }

    fun isReady(): Boolean = state is State.Ready

    private fun calculatePowerForRPM(targetRPM: Double, currentRPM: Double): Double {
        val error = targetRPM - currentRPM
        val power = 0.0014 * error
        return power.coerceIn(0.0, 1.0)
    }

    fun spinUp(): Closure = exec { state = State.SpinningUp(3500.0) }

    fun spinToRPM(targetRPM: Double): Closure = exec { state = State.SpinningUp(targetRPM) }

    fun setPower(power: Double): Closure = exec { state = State.ManualPower(power) }

    fun stop(): Closure = exec { state = State.Off }

    fun spinToRPMDirect(targetRPM: Double) {
        state = State.SpinningUp(targetRPM)
    }
}
