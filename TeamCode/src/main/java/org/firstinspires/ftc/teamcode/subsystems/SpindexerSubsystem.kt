package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import kotlin.math.abs
import kotlin.math.sign

/**
 * Spindexer subsystem for a rotating ball holder with 6 positions.
 *
 * The spindexer holds 3 balls and has 6 positions:
 * - 3 intake positions (0°, 120°, 240°)
 * - 3 outtake positions (60°, 180°, 300°)
 *
 * Uses an Axon Mini+ continuous rotation servo with analog encoder feedback.
 * The analog encoder outputs 0-4.8V for 0-360° on REV Control Hub.
 */
@Inject
@HardwareScoped
@Config
class SpindexerSubsystem(hardwareMap: HardwareMap) : Subsystem() {

    private val servo: CRServo = hardwareMap.get(CRServo::class.java, "spindexer")
    private val encoder: AnalogInput = hardwareMap.get(AnalogInput::class.java, "spindexerEncoder")

    companion object {
        private const val MAX_VOLTAGE = 3.3

        @JvmField
        var TOLERANCE = 18.0 // degrees
    }

    sealed interface State {
        /** Servo stopped, holding position passively */
        object Idle : State

        /** Moving to a target angle */
        data class MovingToPosition(val targetDegrees: Double) : State

        /** Manual power control */
        data class ManualControl(val power: Double) : State
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

                if (abs(currentAngle - s.targetDegrees) <= TOLERANCE) {
                    servo.power = 0.0
                    state = State.Idle
                    return@exec
                }

                val delta = shortestDelta(s.targetDegrees, currentAngle)
                // PID control with proper dt-based derivative

                // Integral term (with dt scaling)
                integral += error * dt
                integral = integral.coerceIn(-RobotConstants.AXON_MAX_INTEGRAL, RobotConstants.AXON_MAX_INTEGRAL)

                // Derivative term (rate of change, with low-pass filter)
                val rawDerivative = if (dt > 0.0001) (error - previousError) / dt else 0.0
                // Low-pass filter: new = alpha * raw + (1-alpha) * old
                // Alpha of 0.2 means 80% old value, 20% new value (smooths out noise)
                filteredDerivative = RobotConstants.AXON_DERIVATIVE_FILTER * rawDerivative +
                        (1.0 - RobotConstants.AXON_DERIVATIVE_FILTER) * filteredDerivative
                previousError = error

                var power = (RobotConstants.AXON_KP * error) +
                        (RobotConstants.AXON_KI * integral) +
                        (RobotConstants.AXON_KD * filteredDerivative)

                // Apply minimum power to overcome static friction (only if error is significant)
                if (abs(power) < RobotConstants.AXON_MIN_POWER && abs(error) > RobotConstants.AXON_DEADBAND) {
                    power = RobotConstants.AXON_MIN_POWER * sign(power)
                }

                currentPower = power.coerceIn(-RobotConstants.AXON_MAX_POWER, RobotConstants.AXON_MAX_POWER)
                servo.power = currentPower
            }

            is State.ManualControl -> {
                servo.power = s.power
            }
        }
    }

// ═══════════════════════════════════════════════════════
// ENCODER READING
// ═══════════════════════════════════════════════════════

    /**
     * Get current angle from the analog encoder.
     * @return Angle in degrees (0-360)
     */
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
     * Rotate spindexer clockwise (positive direction) by 60 degrees.
     */
    fun rotateRight(): Closure = exec {
        val currentAngle = getCurrentAngle()
        state = State.MovingToPosition(normalizeAngle(currentAngle + 60.0))
    }

    /**
     * Rotate spindexer counter-clockwise (negative direction) by 60 degrees.
     */
    fun rotateLeft(): Closure = exec {
        val currentAngle = getCurrentAngle()
        state = State.MovingToPosition(normalizeAngle(currentAngle - 60.0))
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
