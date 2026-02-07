package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.AnalogInput
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareFactory
import org.firstinspires.ftc.teamcode.di.HardwareScope
import kotlin.math.abs

/**
 * Turret Subsystem
 * 
 * Controls a turret mechanism using 2 servos geared together and an analog encoder
 * (REV Through Bore) for position feedback.
 * 
 * The turret has a dead zone around 0/360 degrees where wires prevent free rotation.
 * The subsystem enforces angle limits to prevent wire wrap.
 * 
 * Hardware:
 * - turretServo0: First turret servo
 * - turretServo1: Second turret servo (geared together, same direction)
 * - turretEncoder: REV Through Bore encoder (analog input, 0V-3.3V = 0-360 degrees)
 */
@Config
@Inject
@HardwareScope
class TurretSubsystem(factory: HardwareFactory) : Subsystem<TurretSubsystem.State, TurretSubsystem.Request>() {

    companion object {
        @JvmField var DEAD_ZONE_CENTER = 0.0
        @JvmField var DEAD_ZONE_BUFFER = 15.0

        // MIN_ANGLE = DEAD_ZONE_CENTER + DEAD_ZONE_BUFFER (e.g., 15 degrees)
        // MAX_ANGLE = 360 - DEAD_ZONE_BUFFER (e.g., 345 degrees)

        @JvmField var ENCODER_MIN_VOLTAGE = 0.0
        @JvmField var ENCODER_MAX_VOLTAGE = 3.3
        @JvmField var ENCODER_OFFSET = 0.0
        @JvmField var SERVO_CENTER_POSITION = 0.5  // Servo position when turret is at 180 degrees
        @JvmField var SERVO_DEGREES_PER_UNIT = 300.0  // Degrees of turret rotation per 1.0 servo range
        @JvmField var ANGLE_TOLERANCE = 2.0
        @JvmField var DEFAULT_ANGLE = 180.0
    }

    private val servo0 = factory.getServo("turretServo0")
    private val servo1 = factory.getServo("turretServo1")
    private val encoder: AnalogInput = factory.getAnalogInput("turretEncoder")

    @Volatile var targetAngle: Double = DEFAULT_ANGLE
        private set

    sealed interface State {
        object Idle : State
        data class Targeting(val angle: Double) : State
    }

    sealed interface Request {
        data class SetAngle(val angleDegrees: Double) : Request
        object Hold : Request
    }

    override val initialState = { State.Idle }

    override val transition = { _: State, msg: Request ->
        when (msg) {
            is Request.SetAngle -> {
                val clampedAngle = clampToValidRange(msg.angleDegrees)
                targetAngle = clampedAngle
                State.Targeting(clampedAngle)
            }
            is Request.Hold -> {
                targetAngle = getCurrentAngle()
                State.Idle
            }
        }
    }

    override val behavior = { register: VarRegister<State> ->
        match { register.get() }
            .branch(State.Idle, loop(exec {
                setServosToAngle(targetAngle)
            }))
            .branch({ state: State -> state is State.Targeting } as State, loop(exec {
                val state = register.get() as State.Targeting
                setServosToAngle(state.angle)
                if (isAtTarget()) {
                    register.set(State.Idle)
                }
            }))
            .assertExhaustive()
    }

    /**
     * Command the turret to rotate to a specific angle.
     * The angle will be clamped to the valid range (avoiding the dead zone).
     * 
     * @param angleDegrees Target angle in degrees (0-360)
     */
    fun setAngle(angleDegrees: Double): Closure = update(Request.SetAngle(angleDegrees))

    /**
     * Command the turret to hold its current position.
     */
    fun hold(): Closure = update(Request.Hold)

    /**
     * Get the current turret angle from the encoder.
     * 
     * @return Current angle in degrees (0-360)
     */
    fun getCurrentAngle(): Double {
        val voltage = encoder.voltage
        val normalizedVoltage = (voltage - ENCODER_MIN_VOLTAGE) / 
                                (ENCODER_MAX_VOLTAGE - ENCODER_MIN_VOLTAGE)
        var angle = normalizedVoltage * 360.0 + ENCODER_OFFSET

        angle = ((angle % 360.0) + 360.0) % 360.0
        return angle
    }

    /**
     * Check if the turret is at the target angle (within tolerance).
     */
    fun isAtTarget(): Boolean {
        val error = getAngleError()
        return abs(error) <= ANGLE_TOLERANCE
    }

    /**
     * Get the angular error (difference between current and target).
     * Positive = need to rotate clockwise, Negative = counter-clockwise.
     */
    fun getAngleError(): Double {
        val current = getCurrentAngle()
        var error = targetAngle - current

        while (error > 180) error -= 360
        while (error < -180) error += 360
        
        return error
    }

    /**
     * Get the raw encoder voltage (for debugging/calibration).
     */
    fun getEncoderVoltage(): Double = encoder.voltage

    /**
     * Check if a given angle is in the dead zone.
     */
    fun isInDeadZone(angle: Double): Boolean {
        val normalized = ((angle % 360.0) + 360.0) % 360.0
        val minValid = DEAD_ZONE_CENTER + DEAD_ZONE_BUFFER
        val maxValid = 360.0 - DEAD_ZONE_BUFFER + DEAD_ZONE_CENTER

        return if (DEAD_ZONE_CENTER < DEAD_ZONE_BUFFER) {
            normalized !in minValid..maxValid
        } else {
            normalized > DEAD_ZONE_CENTER - DEAD_ZONE_BUFFER && 
            normalized < DEAD_ZONE_CENTER + DEAD_ZONE_BUFFER
        }
    }

    /**
     * Get the minimum valid angle (edge of dead zone).
     */
    fun getMinValidAngle(): Double = DEAD_ZONE_CENTER + DEAD_ZONE_BUFFER

    /**
     * Get the maximum valid angle (edge of dead zone).
     */
    fun getMaxValidAngle(): Double = 360.0 - DEAD_ZONE_BUFFER + DEAD_ZONE_CENTER

    /**
     * Clamp an angle to the valid range, avoiding the dead zone.
     * If the angle is in the dead zone, snap to the nearest valid edge.
     */
    private fun clampToValidRange(angle: Double): Double {
        var normalized = ((angle % 360.0) + 360.0) % 360.0
        
        val minValid = getMinValidAngle()
        val maxValid = getMaxValidAngle()

        if (isInDeadZone(normalized)) {
            val distToMin = abs(normalizeAngleDiff(normalized - minValid))
            val distToMax = abs(normalizeAngleDiff(normalized - maxValid))
            
            normalized = if (distToMin <= distToMax) minValid else maxValid
        }
        
        return normalized
    }

    /**
     * Normalize an angle difference to -180 to +180.
     */
    private fun normalizeAngleDiff(diff: Double): Double {
        var d = diff
        while (d > 180) d -= 360
        while (d < -180) d += 360
        return d
    }

    /**
     * Convert a target angle to servo position and command both servos.
     */
    private fun setServosToAngle(angleDegrees: Double) {
        val angleFromCenter = angleDegrees - 180.0
        val servoOffset = angleFromCenter / SERVO_DEGREES_PER_UNIT
        val servoPosition = (SERVO_CENTER_POSITION + servoOffset).coerceIn(0.0, 1.0)

        servo0.position = servoPosition
        servo1.position = servoPosition
    }
}
