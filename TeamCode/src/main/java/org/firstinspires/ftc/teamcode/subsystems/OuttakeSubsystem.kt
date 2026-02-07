package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareFactory
import org.firstinspires.ftc.teamcode.di.HardwareScope
import kotlin.math.abs

@Config
@Inject
@HardwareScope
class OuttakeSubsystem(factory: HardwareFactory) : Subsystem<OuttakeSubsystem.State, OuttakeSubsystem.Request>() {

    companion object {
        @JvmField
        var MIN_HOOD_ANGLE = 30.0

        @JvmField
        var MAX_HOOD_ANGLE = 75.0

        @JvmField
        var DEFAULT_HOOD_ANGLE = 45.0

        @JvmField
        var RPM_TOLERANCE = 50.0

        @JvmField
        var TICKS_PER_REV = 28.0

        @JvmField
        var MAX_RPM = 5000.0
    }

    private val motor0: DcMotorEx = factory.getMotor("outtakeMotor0").apply {
        mode = DcMotor.RunMode.RUN_USING_ENCODER
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    }

    private val motor1: DcMotorEx = factory.getMotor("outtakeMotor1").apply {
        mode = DcMotor.RunMode.RUN_USING_ENCODER
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        direction = DcMotorSimple.Direction.REVERSE
    }

    private val hoodServo = factory.getServo("outtakeHood")

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
        object Stop : Request
    }

    override val initialState = { State.Idle }

    override val transition = { _: State, msg: Request ->
        when (msg) {
            is Request.SetTarget -> {
                targetRPM = msg.rpm.coerceIn(0.0, MAX_RPM)
                targetHoodAngle = msg.hoodAngleDegrees.coerceIn(MIN_HOOD_ANGLE, MAX_HOOD_ANGLE)
                State.Spinning(targetRPM, targetHoodAngle)
            }

            is Request.Stop -> {
                targetRPM = 0.0
                targetHoodAngle = DEFAULT_HOOD_ANGLE
                State.Idle
            }
        }
    }

    override val behavior = { register: VarRegister<State> ->
        match { register.get() }.branch(State.Idle, exec {
            motor0.power = 0.0
            motor1.power = 0.0
            setHoodServoPosition(DEFAULT_HOOD_ANGLE)
        }).branch({ state: State -> state is State.Spinning } as State, loop(exec {
            val state = register.get() as State.Spinning
            val power = (state.rpm / MAX_RPM).coerceIn(0.0, 1.0)
            motor0.power = power
            motor1.power = power
            setHoodServoPosition(state.hoodAngleDegrees)
        })).assertExhaustive()
    }

    /**
     * Command the outtake to spin up to target RPM and set hood angle.
     * @param rpm Target RPM (0 to MAX_RPM)
     * @param hoodAngleDegrees Hood angle in degrees (MIN_HOOD_ANGLE to MAX_HOOD_ANGLE)
     */
    fun spinUp(rpm: Double, hoodAngleDegrees: Double): Closure = update(Request.SetTarget(rpm, hoodAngleDegrees))

    /**
     * Command the outtake to spin up to target RPM using default hood angle.
     * @param rpm Target RPM (0 to MAX_RPM)
     */
    fun spinUp(rpm: Double): Closure = update(Request.SetTarget(rpm, DEFAULT_HOOD_ANGLE))

    /**
     * Stop the outtake motors and reset hood to default position.
     */
    fun stop(): Closure = update(Request.Stop)

    /**
     * Get the current average RPM of both shooter motors.
     * Uses motor velocity feedback converted to RPM.
     */
    fun getCurrentRPM(): Double {
        val velocity0 = motor0.velocity
        val velocity1 = abs(motor1.velocity)

        val rpm0 = (velocity0 / TICKS_PER_REV) * 60.0
        val rpm1 = (velocity1 / TICKS_PER_REV) * 60.0

        return (rpm0 + rpm1) / 2.0
    }

    /**
     * Get the current RPM of motor 0.
     */
    fun getMotor0RPM(): Double {
        return (motor0.velocity / TICKS_PER_REV) * 60.0
    }

    /**
     * Get the current RPM of motor 1.
     */
    fun getMotor1RPM(): Double {
        return (abs(motor1.velocity) / TICKS_PER_REV) * 60.0
    }

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

    /**
     * Set the hood servo to the specified angle in degrees.
     */
    private fun setHoodServoPosition(angleDegrees: Double) {
        hoodServo.position = angleToServoPosition(angleDegrees)
    }
}
