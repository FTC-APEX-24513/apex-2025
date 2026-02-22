package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.matchType
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareFactory
import org.firstinspires.ftc.teamcode.di.HardwareScope
import org.firstinspires.ftc.teamcode.hardware.MotorGroup
import org.firstinspires.ftc.teamcode.hardware.ServoGroup
import org.firstinspires.ftc.teamcode.di.group
import kotlin.math.abs

@Config
@Inject
@HardwareScope
class OuttakeSubsystem(factory: HardwareFactory) : Subsystem<OuttakeSubsystem.State, OuttakeSubsystem.Request>() {

    companion object {
        @JvmField
        var MIN_HOOD_ANGLE = 20.0

        @JvmField
        var MAX_HOOD_ANGLE = 50.0

        @JvmField
        var DEFAULT_HOOD_ANGLE = 43.0

        @JvmField
        var RPM_TOLERANCE = 50.0

        @JvmField
        var TICKS_PER_REV = 28.0

        @JvmField
        var MAX_RPM = 5000.0
    }

    private val flywheel = factory.group<DcMotorEx>(
        "outtake0" to DcMotorSimple.Direction.FORWARD,
        "outtake1" to DcMotorSimple.Direction.REVERSE,
    ).also {
        it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    } as MotorGroup


    private val hood = factory.group<Servo>(
        "hood0" to Servo.Direction.FORWARD,
        "hood1" to Servo.Direction.REVERSE,
    ) as ServoGroup

    @Volatile
    var targetRPM: Double = 0.0
        private set

    @Volatile
    var targetHoodAngle: Double = DEFAULT_HOOD_ANGLE
        private set


    sealed interface State {
        object Idle : State
        data class Spinning(val rpm: Double, val hoodAngleDegrees: Double) : State
    }

    sealed interface Request {
        data class SetTarget(val rpm: Double, val hoodAngleDegrees: Double) : Request
        data class AdjustHood(val angleDegrees: Double) : Request
        object Stop : Request
    }

    override val initialState = { State.Idle }

    override val transition = { current: State, msg: Request ->
        when (msg) {
            is Request.SetTarget -> {
                targetRPM = msg.rpm.coerceIn(0.0, MAX_RPM)
                targetHoodAngle = msg.hoodAngleDegrees.coerceIn(MIN_HOOD_ANGLE, MAX_HOOD_ANGLE)
                State.Spinning(targetRPM, targetHoodAngle)
            }

            is Request.AdjustHood -> {
                targetHoodAngle = msg.angleDegrees.coerceIn(MIN_HOOD_ANGLE, MAX_HOOD_ANGLE)
                when (current) {
                    is State.Spinning -> State.Spinning(current.rpm, targetHoodAngle)
                    is State.Idle     -> State.Idle
                }
            }

            is Request.Stop -> {
                targetRPM = 0.0
                // Hood angle is intentionally preserved so the servo holds position while idle.
                State.Idle
            }
        }
    }

    override val behavior = { register: VarRegister<State> ->
        matchType({ register.get() })
            .branch<State.Idle>(exec {
                flywheel.power = 0.0
                // Track targetHoodAngle so adjustHood() works even while not spinning.
                hood.position = angleToServoPosition(targetHoodAngle)
            })
            .branch<State.Spinning> { stateReg ->
                loop(exec {
                    val state = stateReg.get()
                    flywheel.power = (state.rpm / MAX_RPM).coerceIn(0.0, 1.0)
                    hood.position = angleToServoPosition(state.hoodAngleDegrees)
                })
            }
            .assertExhaustive()
    }

    /**
     * Command the outtake to spin up to target RPM and set hood angle.
     * @param rpm Target RPM (0 to MAX_RPM)
     * @param hoodAngleDegrees Hood angle in degrees (MIN_HOOD_ANGLE to MAX_HOOD_ANGLE)
     */
    fun spinUp(rpm: Double, hoodAngleDegrees: Double): Closure = update(Request.SetTarget(rpm, hoodAngleDegrees))

    /**
     * Lazy overload — reads [rpm] and [hoodAngleDegrees] at send time, not at registration time.
     * Use when the values change tick-to-tick (e.g. a Kotlin var updated by gamepad input).
     */
    fun spinUp(rpm: () -> Double, hoodAngleDegrees: () -> Double): Closure =
        Channels.send({ Request.SetTarget(rpm(), hoodAngleDegrees()) }, { actor.tx })

    /**
     * Command the outtake to spin up to target RPM using default hood angle.
     * @param rpm Target RPM (0 to MAX_RPM)
     */
    fun spinUp(rpm: Double): Closure = update(Request.SetTarget(rpm, DEFAULT_HOOD_ANGLE))

    /**
     * Adjust the hood angle without changing the flywheel RPM.
     * If the outtake is Spinning, updates the hood in-place.
     * If Idle, the angle is stored and will apply on the next spinUp.
     * @param angleDegrees Absolute hood angle (MIN_HOOD_ANGLE to MAX_HOOD_ANGLE)
     */
    fun adjustHood(angleDegrees: Double): Closure = update(Request.AdjustHood(angleDegrees))

    /**
     * Lazy overload — evaluates [angleDegrees] at send time.
     */
    fun adjustHood(angleDegrees: () -> Double): Closure =
        Channels.send({ Request.AdjustHood(angleDegrees()) }, { actor.tx })

    /**
     * Stop the outtake motors and reset hood to default position.
     */
    fun stop(): Closure = update(Request.Stop)

    /**
     * Get the current average RPM of both shooter motors.
     * Uses motor velocity feedback converted to RPM.
     */
    fun getCurrentRPM(): Double {
        val rpm0 = (flywheel[0].velocity / TICKS_PER_REV) * 60.0
        val rpm1 = (abs(flywheel[1].velocity) / TICKS_PER_REV) * 60.0
        return (rpm0 + rpm1) / 2.0
    }

    /**
     * Get the current RPM of motor 0.
     */
    fun getMotor0RPM(): Double = (flywheel[0].velocity / TICKS_PER_REV) * 60.0

    /**
     * Get the current RPM of motor 1.
     */
    fun getMotor1RPM(): Double = (abs(flywheel[1].velocity) / TICKS_PER_REV) * 60.0

    /**
     * Check if the outtake is at the target RPM (within tolerance).
     */
    fun isAtTargetRPM(): Boolean {
        if (targetRPM == 0.0) return true
        return abs(getCurrentRPM() - targetRPM) <= RPM_TOLERANCE
    }

    /**
     * Get the current hood angle in degrees.
     */
    fun getCurrentHoodAngle(): Double = targetHoodAngle

    /**
     * Convert hood angle in degrees to servo position (0.0 to 1.0).
     */
    private fun angleToServoPosition(angleDegrees: Double): Double {
        val clampedAngle = angleDegrees.coerceIn(MIN_HOOD_ANGLE, MAX_HOOD_ANGLE)
        return (clampedAngle - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE)
    }
}
