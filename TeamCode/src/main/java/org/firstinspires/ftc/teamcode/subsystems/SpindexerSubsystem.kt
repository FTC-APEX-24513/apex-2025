package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import org.firstinspires.ftc.teamcode.util.VoltageCompensation
import kotlin.math.abs
import kotlin.math.sign

@Inject
@HardwareScoped
@Config
class SpindexerSubsystem(hardwareMap: HardwareMap, private val voltageCompensation: VoltageCompensation) : Subsystem() {

    private val servo: CRServo = hardwareMap.get(CRServo::class.java, "spindexer")
    private val encoder: AnalogInput = hardwareMap.get(AnalogInput::class.java, "spindexerEncoder")

    // PID State
    private val timer = ElapsedTime()
    private var lastError = 0.0
    private var integralSum = 0.0

    companion object {
        private const val MAX_VOLTAGE = 3.3

        @JvmField
        var TOLERANCE = 18.0 // degrees

        @JvmField
        var kP = 0.015 // Proportional: Start small
        @JvmField
        var kI = 0.0   // Integral: Fixes steady-state error
        @JvmField
        var kD = 0.0   // Derivative: Dampens oscillation

        @JvmField
        var MAX_INTEGRAL_SUM = 1000.0

        // Time-based spinning constants (from test teleop)
        @JvmField
        var FULL_ROTATION_TIME_MS = 800.0 // Time for full 360° rotation

        @JvmField
        var SERVO_POWER = 1.0 // Power for time-based spinning
    }

    sealed interface State {
        /** Servo stopped, holding position passively */
        object Idle : State

        /** Moving to a target angle */
        data class MovingToPosition(val targetDegrees: Double) : State

        /** Manual power control */
        data class ManualControl(val power: Double) : State

        /** Time-based spinning for X milliseconds */
        data class SpinningTimed(val power: Double, val durationMs: Double, val startTimeMs: Double) : State
    }

    var state: State = State.Idle
        private set

    override fun periodic(): Closure = exec {
        when (val s = state) {
            is State.Idle -> {
                servo.power = 0.0
            }

            is State.MovingToPosition -> {
                val currentAngle = getCurrentAngle()
                val error = shortestDelta(s.targetDegrees, currentAngle)
                val dt = timer.seconds()
                timer.reset() // Reset for next loop

                if (abs(error) <= TOLERANCE) {
                    servo.power = 0.0
                    state = State.Idle
                    return@exec
                }

                // PID Calculations
                // Avoid division by zero if periodic runs instantly (unlikely but safe)
                val safeDt = if (dt > 0.0) dt else 0.001
                val derivative = (error - lastError) / safeDt

                // Integral with windup protection
                if (sign(error) != sign(lastError)) {
                    integralSum = 0.0 // Zero crossing reset
                }
                integralSum = (integralSum + (error * safeDt)).coerceIn(-MAX_INTEGRAL_SUM, MAX_INTEGRAL_SUM)

                val output = (error * kP) + (integralSum * kI) + (derivative * kD) * ((currentAngle - s.targetDegrees) / s.targetDegrees)

                servo.power = voltageCompensation.compensate(output.coerceIn(-1.0, 1.0))

                lastError = error
            }

            is State.ManualControl -> {
                servo.power = voltageCompensation.compensate(s.power)
            }

            is State.SpinningTimed -> {
                val elapsedMs = timer.milliseconds() - s.startTimeMs
                if (elapsedMs >= s.durationMs) {
                    servo.power = 0.0
                    state = State.Idle
                } else {
                    servo.power = voltageCompensation.compensate(s.power)
                }
            }
        }
    }

    // ═══════════════════════════════════════════════════════
    // ENCODER READING
    // ═══════════════════════════════════════════════════════

    fun getNormalizedPosition(): Double {
        val voltage = encoder.voltage
        // TODO: Verify if the encoder output is 0-3.3V or 0-4.8V. 
        // Current logic assumes 3.3V max. If 4.8V, this needs a voltage divider or logic change.
        return (voltage / MAX_VOLTAGE)
    }

    fun getCurrentAngle(): Double {
        val voltage = encoder.voltage
        return (voltage / MAX_VOLTAGE) * 360.0
    }
    /**
     * Calculate the shortest angle difference (accounting for wraparound).
     * @return Signed difference from current to target (-180 to 180)
     */
    private fun shortestDelta(target: Double, current: Double): Double {
        var diff = target - current
        while (diff > 180) diff -= 360
        while (diff < -180) diff += 360
        return diff
    }

// ═══════════════════════════════════════════════════════
// POSITION COMMANDS
// ═══════════════════════════════════════════════════════

    /**
     * Helper to reset PID and set state.
     */
    private fun startMove(target: Double) {
        timer.reset()
        lastError = 0.0
        integralSum = 0.0
        state = State.MovingToPosition(normalizeAngle(target))
    }

    /**
     * Rotate spindexer clockwise (positive direction) by 60 degrees using time-based control.
     * Uses FULL_ROTATION_TIME_MS and SERVO_POWER from companion object for tuning.
     */
    fun rotateRight(): Closure = exec {
        val time60Degrees = FULL_ROTATION_TIME_MS / 6.0
        state = State.SpinningTimed(SERVO_POWER, time60Degrees, timer.milliseconds())
    }

    /**
     * Rotate spindexer counter-clockwise (negative direction) by 60 degrees using time-based control.
     * Uses FULL_ROTATION_TIME_MS and SERVO_POWER from companion object for tuning.
     */
    fun rotateLeft(): Closure = exec {
        val time60Degrees = FULL_ROTATION_TIME_MS / 6.0
        state = State.SpinningTimed(-SERVO_POWER, time60Degrees, timer.milliseconds())
    }

    /**
     * Set manual power control.
     * @param power Power from -1.0 to 1.0 (positive = one direction, negative = other)
     */
    fun setManualPower(power: Double): Closure = exec {
        state = State.ManualControl(power.coerceIn(-1.0, 1.0))
    }

    /**
     * Stop the spindexer.
     */
    fun stop(): Closure = exec {
        state = State.Idle
    }

// ═══════════════════════════════════════════════════════
// INTERNAL HELPERS
// ═══════════════════════════════════════════════════════

    private fun normalizeAngle(angle: Double): Double {
        var normalized = angle % 360.0
        if (normalized < 0) normalized += 360.0
        return normalized
    }
}
