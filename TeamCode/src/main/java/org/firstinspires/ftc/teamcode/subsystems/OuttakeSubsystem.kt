package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import org.firstinspires.ftc.teamcode.physics.ShootingCalculator
import kotlin.math.abs

@Config
@Inject
@HardwareScoped
class OuttakeSubsystem(hardwareMap: HardwareMap) : Subsystem() {
    private val motor = hardwareMap.get(DcMotorEx::class.java, "flywheel").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    companion object {
        // Robot Physical
        @JvmField var LAUNCH_HEIGHT_METERS = 0.2255
        @JvmField var LAUNCH_ANGLE_RADIANS = 0.0

        // Flywheel Physical
        @JvmField var FLYWHEEL_RADIUS_METERS = 0.060
        @JvmField var FLYWHEEL_MASS_KG = 0.150
        @JvmField var FLYWHEEL_INNER_RADIUS_METERS = 0.055

        val FLYWHEEL_MOMENT_OF_INERTIA: Double
            get() = 0.5 * FLYWHEEL_MASS_KG * 
                (FLYWHEEL_RADIUS_METERS * FLYWHEEL_RADIUS_METERS + 
                 FLYWHEEL_INNER_RADIUS_METERS * FLYWHEEL_INNER_RADIUS_METERS)

        // Ball Physical
        @JvmField var BALL_MASS_KG = 0.065
        @JvmField var BALL_CONTACT_ARC_DEGREES = 45.0

        // Strategy
        @JvmField var MAX_SHOT_DISTANCE_METERS = 3.5
        @JvmField var MIN_SHOT_DISTANCE_METERS = 0.3
        
        // Feedback Zones
        @JvmField var SHOOTING_ZONE_MIN_METERS = 1.0
        @JvmField var SHOOTING_ZONE_MAX_METERS = 2.5
        @JvmField var SHOOTING_ZONE_OPTIMAL_MIN = 1.5
        @JvmField var SHOOTING_ZONE_OPTIMAL_MAX = 2.0

        // Control
        @JvmField var FLYWHEEL_TO_BALL_EFFICIENCY = 0.87
        @JvmField var MAX_FLYWHEEL_RPM = 5000.0
        @JvmField var MIN_STABLE_RPM = 500.0
        @JvmField var TICKS_PER_REVOLUTION = 28.0
        @JvmField var RPM_PROPORTIONAL_GAIN = 0.000125
        @JvmField var DEFAULT_SHOOTING_RPM = 3500.0
        @JvmField var LAUNCH_DURATION = 0.3

        @JvmField var MOTOR_STALL_TORQUE_NM = 0.170
        @JvmField var MOTOR_GEAR_RATIO = 1.0

        val MOTOR_ACCELERATION_RATE_RPM_PER_SEC: Double 
            get() = (MOTOR_STALL_TORQUE_NM / FLYWHEEL_MOMENT_OF_INERTIA) * (60.0 / (2.0 * kotlin.math.PI)) * MOTOR_GEAR_RATIO

        // Recovery
        @JvmField var MIN_RECOVERY_TIME_SECONDS = 0.15
        @JvmField var RECOVERY_TIME_SAFETY_FACTOR = 1.2
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

    var lockedRPM: Double? = null
        private set

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
        val revolutionsPerSecond = ticksPerSecond / TICKS_PER_REVOLUTION
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
        val power = RPM_PROPORTIONAL_GAIN * error
        return power.coerceIn(0.0, 1.0)
    }

    // ═══════════════════════════════════════════════════════
    // COMMANDS
    // ═══════════════════════════════════════════════════════

    fun spinUp(): Closure = exec { state = State.SpinningUp(DEFAULT_SHOOTING_RPM) }
    
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

    fun lockToDistance(distance: Double): Closure = exec {
        lockToDistanceDirect(distance)
    }

    fun lockToDistanceDirect(distance: Double) {
        val params = ShootingCalculator.calculateShotParameters(distance)
        if (params != null) {
            lockedRPM = params.targetRPM
            lockedDistance = distance
            state = State.SpinningUp(params.targetRPM)
        } else {
            lockedRPM = DEFAULT_SHOOTING_RPM
            lockedDistance = distance
            state = State.SpinningUp(DEFAULT_SHOOTING_RPM)
        }
    }

    fun spinToRPMDirect(targetRPM: Double) {
        lockedRPM = targetRPM
        lockedDistance = null
        state = State.SpinningUp(targetRPM)
    }

    fun launchAndRecover(): Closure {
        val recoveryRPM = lockedRPM
        return sequence(
            exec { state = State.Launching },
            wait(LAUNCH_DURATION),
            exec { 
                state = if (recoveryRPM != null) {
                    State.SpinningUp(recoveryRPM)
                } else {
                    State.Off
                }
            }
        )
    }

    fun launch(): Closure = sequence(
        exec { state = State.Launching },
        wait(LAUNCH_DURATION),
        exec { state = State.Off }
    )
}
