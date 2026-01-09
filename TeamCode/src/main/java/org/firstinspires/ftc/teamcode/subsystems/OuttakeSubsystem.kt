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
import org.firstinspires.ftc.teamcode.constants.ShootingConstants
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import org.firstinspires.ftc.teamcode.physics.ShootingCalculator
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

    /** Last locked target RPM (for telemetry) */
    var lockedRPM: Double? = null
        private set

    /** Last distance used to calculate RPM (for telemetry) */
    var lockedDistance: Double? = null
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

    /** Check if flywheel is ready to fire (at target speed) */
    fun isReady(): Boolean = state is State.Ready

    private fun calculatePowerForRPM(targetRPM: Double, currentRPM: Double): Double {
        val error = targetRPM - currentRPM
        val power = RobotConstants.OUTTAKE_RPM_PROPORTIONAL_GAIN * error
        return power.coerceIn(0.0, 1.0)
    }

    // ═══════════════════════════════════════════════════════
    // COMMANDS
    // ═══════════════════════════════════════════════════════

    fun spinUp(): Closure = exec { state = State.SpinningUp(RobotConstants.OUTTAKE_DEFAULT_SPINUP_RPM) }
    
    fun spinToRPM(targetRPM: Double): Closure = exec { 
        lockedRPM = targetRPM
        lockedDistance = null
        state = State.SpinningUp(targetRPM) 
    }
    
    fun setPower(power: Double): Closure = exec { state = State.ManualPower(power) }
    
    fun stop(): Closure = exec { 
        lockedRPM = null
        lockedDistance = null
        state = State.Off 
    }

    /**
     * Lock RPM based on distance to goal.
     * Calculates required RPM using physics and spins up.
     * 
     * @param distance Distance to goal in meters
     * @return Closure that starts spinup, or stops if shot not feasible
     */
    fun lockToDistance(distance: Double): Closure = exec {
        lockToDistanceDirect(distance)
    }

    /**
     * Direct method to lock RPM based on distance (for use in exec blocks).
     */
    fun lockToDistanceDirect(distance: Double) {
        val params = ShootingCalculator.calculateShotParameters(distance)
        if (params != null) {
            lockedRPM = params.targetRPM
            lockedDistance = distance
            state = State.SpinningUp(params.targetRPM)
        } else {
            // Shot not feasible - use default RPM as fallback
            lockedRPM = ShootingConstants.DEFAULT_SHOOTING_RPM
            lockedDistance = distance
            state = State.SpinningUp(ShootingConstants.DEFAULT_SHOOTING_RPM)
        }
    }

    /**
     * Direct method to spin to RPM (for use in exec blocks).
     */
    fun spinToRPMDirect(targetRPM: Double) {
        lockedRPM = targetRPM
        lockedDistance = null
        state = State.SpinningUp(targetRPM)
    }

    /**
     * Launch sequence - fires then returns to spinning at locked RPM.
     * If no locked RPM, just fires and stops.
     */
    fun launchAndRecover(): Closure {
        val recoveryRPM = lockedRPM
        return sequence(
            exec { state = State.Launching },
            wait(RobotConstants.OUTTAKE_LAUNCH_DURATION),
            exec { 
                state = if (recoveryRPM != null) {
                    State.SpinningUp(recoveryRPM)
                } else {
                    State.Off
                }
            }
        )
    }

    /**
     * Simple launch - fires then stops.
     */
    fun launch(): Closure = sequence(
        exec { state = State.Launching },
        wait(RobotConstants.OUTTAKE_LAUNCH_DURATION),
        exec { state = State.Off }
    )
}
