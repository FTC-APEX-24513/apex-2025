package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.matchType
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareFactory
import org.firstinspires.ftc.teamcode.di.HardwareScope
import org.firstinspires.ftc.teamcode.di.group
import org.firstinspires.ftc.teamcode.hardware.CRServoGroup
import kotlin.math.abs
import kotlin.math.sign

/**
 * Turret Subsystem — Limelight-guided, encoder-free, CR servo drive.
 *
 * Without an encoder the turret uses dead-reckoning to estimate its position.
 * Every tick that power is applied, the subsystem integrates:
 *
 *     estimatedPosition += power * dt * DEGREES_PER_SECOND
 *
 * Soft limits (LIMIT_MIN, LIMIT_MAX) protect the wire wrap for all automated states
 * (Tracking, Searching, Locked, Idle).  The Driving state bypasses limits so the
 * operator always has full manual override.  Auto-tracking reverses direction at the
 * limit rather than stopping, so the goal is always reachable via the other arc.
 *
 * States
 * ------
 *   Idle       — power = 0; dt still consumed so the ticker stays accurate.
 *   Tracking   — tag visible; power = tx * TX_GAIN, clamped to MAX_POWER.
 *   Locked     — tx within TX_TOLERANCE; power = 0.
 *   Searching  — tag lost; slow sweep in [direction].
 *   Driving    — raw manual drive from operator input.
 *
 * Requests
 * --------
 *   Track(tx)            — tag visible with this horizontal offset; drive toward 0.
 *   Search(direction)    — tag lost; sweep in given direction (±1.0).
 *   Drive(power)         — raw power, operator-driven.
 *   Hold                 — cut power immediately; enter Idle.
 *
 * Hardware
 * --------
 *   turretServo0 / turretServo1 — GoBILDA speed CR servos, geared together.
 *
 * Tuning
 * ------
 *   1. Set DEGREES_PER_SECOND by timing how long a full-power run takes to cover
 *      a known angle, then computing angle / seconds.
 *   2. Set MIN_POWER just above the static-friction threshold so the turret always
 *      moves even for tiny tx errors (typically 0.05–0.15).
 *   3. Set FULL_POWER_TX to the tx error (degrees) at which full MAX_POWER is applied
 *      (typically 20–40°).  Errors between 0 and FULL_POWER_TX are linearly interpolated
 *      between MIN_POWER and MAX_POWER.
 *   4. Set TX_DERIVATIVE_GAIN to damp oscillation.  Start at 0 and increase until
 *      overshoot disappears.  Too high causes sluggish response or chatter.
 *
 * @see TurretSubsystemLegacy for the original position-servo + encoder implementation.
 */
@Config
@Inject
@HardwareScope
class TurretSubsystem(factory: HardwareFactory) : Subsystem<TurretSubsystem.State, TurretSubsystem.Request>() {

    companion object {
        /**
         * Turret rotation speed at servo power = 1.0, in degrees per second.
         * Measured: GoBILDA 2000-0025-0003 at 94.2 RPM (5V / REV Control Hub),
         * through 60t → 15t → 20t → 216t gear train = 209 deg/sec at full power.
         */
        @JvmField var DEGREES_PER_SECOND = 209.0

        /**
         * Soft limit — minimum allowed estimated position (degrees from boot).
         * -90.0 = 90° counter-clockwise from start.
         */
        @JvmField var LIMIT_MIN = -90.0

        /**
         * Soft limit — maximum allowed estimated position (degrees from boot).
         * +180.0 = 180° clockwise from start.
         */
        @JvmField var LIMIT_MAX = 180.0

        /** tx below this threshold (degrees) is considered locked; power cuts to 0. */
        @JvmField var TX_TOLERANCE = 1.5

        /**
         * Minimum power applied for any nonzero tx error.
         * Set just above the static-friction threshold so the turret always moves.
         * Output is linearly interpolated from MIN_POWER (at TX_TOLERANCE) to MAX_POWER
         * (at FULL_POWER_TX), then clamped at MAX_POWER beyond that.
         */
        @JvmField var MIN_POWER = 0.10

        /**
         * tx error magnitude (degrees) at which output reaches MAX_POWER.
         * Errors larger than this saturate at MAX_POWER.
         */
        @JvmField var FULL_POWER_TX = 30.0

        /** Maximum tracking power (0.0–1.0). */
        @JvmField var MAX_POWER = 1.0

        /**
         * Derivative gain on tx rate-of-change.
         * Subtracts TX_DERIVATIVE_GAIN * (dtx/dt) from the proportional output,
         * braking the turret as it converges and damping oscillation.
         * Units: power per (degree/second).  Start at 0.0 and increase until
         * overshoot is eliminated (typical range 0.005–0.05).
         */
        @JvmField var TX_DERIVATIVE_GAIN = 0.01

        /**
         * Maximum Limelight result staleness (ms) for which tracking is allowed.
         * If the latest result is older than this, lockOnGoal sends Hold instead of
         * Track so the turret doesn't run on stale data while the camera catches up.
         */
        @JvmField var MAX_TRACKING_STALENESS_MS = 100L

        /** Power used while searching. Low so the turret sweeps slowly and doesn't overshoot. */
        @JvmField var SEARCH_POWER = 0.5
    }

    private val servos = (factory.group<CRServo>("turret0", "turret1") as CRServoGroup).also {
        it[0].direction = DcMotorSimple.Direction.REVERSE
        it[1].direction = DcMotorSimple.Direction.REVERSE
    }

    // -------------------------------------------------------------------------
    // Dead-reckoning state
    // -------------------------------------------------------------------------

    /**
     * Estimated turret position in degrees from the boot position.
     * 0.0 = wherever the turret was when the OpMode started.
     * Positive = rotated clockwise, negative = counter-clockwise.
     */
    @Volatile var estimatedPosition: Double = 0.0
        private set

    /** True when the turret is at or past a soft limit in either direction. */
    val isAtLimit: Boolean
        get() = estimatedPosition >= LIMIT_MAX || estimatedPosition <= LIMIT_MIN

    // Previous tx and elapsed-time timer for the derivative term.
    // Subsystem-level so they survive actor fiber restarts on each new Track message.
    private var prevTx: Double = 0.0
    private val derivativeTicker = ElapsedTime()

    private val ticker = ElapsedTime()

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------

    sealed interface State {
        object Idle    : State
        data class Tracking(val tx: Double) : State
        object Locked  : State
        /** [direction] is +1.0 (clockwise) or -1.0 (counter-clockwise). */
        data class Searching(val direction: Double) : State
        /** Raw manual drive — bypasses tx math. */
        data class Driving(val power: Double) : State
    }

    sealed interface Request {
        data class Track(val tx: Double) : Request
        /** [lastKnownDirection] = sign of the last non-zero tx (+1.0 or -1.0). */
        data class Search(val lastKnownDirection: Double) : Request
        /** Raw power drive — bypasses tx math. */
        data class Drive(val power: Double) : Request
        object Hold : Request
    }

    override val initialState = { State.Idle }

    override val transition = { current: State, msg: Request ->
        when (msg) {
            is Request.Track  -> {
                if (abs(msg.tx) <= TX_TOLERANCE) State.Locked
                else State.Tracking(msg.tx)
            }
            is Request.Search -> {
                // Don't restart a sweep already in progress.
                if (current is State.Searching) current
                else {
                    val dir = sign(msg.lastKnownDirection).let { if (it == 0.0) 1.0 else it }
                    State.Searching(dir)
                }
            }
            is Request.Drive  -> State.Driving(msg.power.coerceIn(-1.0, 1.0))
            is Request.Hold   -> State.Idle
        }
    }

    override val behavior = { register: VarRegister<State> ->
        // Reset ticker when the actor starts so the first dt isn't huge.
        ticker.reset()

        matchType { register.get() }
            .branch<State.Idle>(loop(exec {
                prevTx = 0.0
                derivativeTicker.reset()
                applyPower(0.0)
            }))
            .branch<State.Locked>(loop(exec {
                prevTx = 0.0
                derivativeTicker.reset()
                applyPower(0.0)
            }))
            .branch<State.Tracking> { stateReg ->
                loop(exec {
                    val tx = stateReg.get().tx
                    val dt = derivativeTicker.seconds().also { derivativeTicker.reset() }
                    // Rate of tx change (degrees/sec). Positive = error growing, negative = converging.
                    val txRate = if (dt > 0.0) (tx - prevTx) / dt else 0.0
                    prevTx = tx
                    val requested = trackingPower(tx, txRate)
                    applyPower(requested)
                    if (abs(tx) <= TX_TOLERANCE) register.set(State.Locked)
                })
            }
            .branch<State.Driving> { stateReg ->
                loop(exec {
                    // Manual drive: limits deliberately not enforced — operator has full override.
                    applyPower(stateReg.get().power, respectLimits = false)
                })
            }
            .branch<State.Searching> { stateReg ->
                loop(exec {
                    val state = stateReg.get()
                    val actual = applyPower(state.direction * SEARCH_POWER)
                    // If power was clamped to 0 the limit in this direction was hit — reverse.
                    if (actual == 0.0 && state.direction * SEARCH_POWER != 0.0) {
                        register.set(State.Searching(-state.direction))
                    }
                })
            }
            .assertExhaustive()
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /**
     * PD power output for tracking.
     *
     * Proportional term — piecewise-linear on |tx|:
     *   [0, TX_TOLERANCE]             → 0           (dead-zone → Locked)
     *   [TX_TOLERANCE, FULL_POWER_TX] → lerp(MIN_POWER, MAX_POWER)
     *   [FULL_POWER_TX, ∞)            → MAX_POWER   (saturated)
     *
     * Derivative term — subtracts TX_DERIVATIVE_GAIN × (dtx/dt) from the P output:
     *   • txRate negative (tx shrinking, turret converging) → D is positive →
     *     subtracted from P → turret brakes before reaching centre.
     *   • txRate positive (tx growing, turret overshooting) → D is negative →
     *     subtracted adds to P in the correcting direction.
     *
     * Final output is clamped to [-MAX_POWER, MAX_POWER].
     */
    private fun trackingPower(tx: Double, txRate: Double): Double {
        val absTx = abs(tx)
        if (absTx <= TX_TOLERANCE) return 0.0
        val t = ((absTx - TX_TOLERANCE) / (FULL_POWER_TX - TX_TOLERANCE)).coerceIn(0.0, 1.0)
        val p = sign(tx) * (MIN_POWER + t * (MAX_POWER - MIN_POWER))
        val d = TX_DERIVATIVE_GAIN * txRate
        return (p - d).coerceIn(-MAX_POWER, MAX_POWER)
    }

    /**
     * Apply power to the servos and integrate the dead-reckoning position estimate.
     *
     * @param power          Desired power in [-1.0, 1.0].
     * @param respectLimits  If true (default), clamps power to 0 when a soft limit is reached.
     *                       Pass false for manual (Driving) state so the operator always has
     *                       full control regardless of the estimated position.
     * @return               The power actually applied (may differ from [power] if a limit
     *                       was hit).  The Searching branch uses this to detect limit contact.
     */
    private fun applyPower(power: Double, respectLimits: Boolean = true): Double {
        val dt = ticker.seconds().also { ticker.reset() }
        val actual = if (respectLimits) {
            when {
                power > 0.0 && estimatedPosition >= LIMIT_MAX -> 0.0
                power < 0.0 && estimatedPosition <= LIMIT_MIN -> 0.0
                else -> power
            }
        } else {
            power
        }
        estimatedPosition += actual * dt * DEGREES_PER_SECOND
        servos.power = actual
        return actual
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /**
     * Tag visible with this tx. Drives toward tx = 0, transitions to Locked when centred.
     *
     * To feed a live value each tick, keep tx in a scoped variable and call this from an
     * exec {} that runs after the variable is updated:
     *   scope { var tx by variable { 0.0 }; sequence(exec { tx = read() }, turret.track(tx)) }
     * The scoped register ensures the value passed here is whatever was written this tick.
     */
    fun track(tx: Double): Closure = Channels.send({ Request.Track(tx) }, { actor.tx })

    /**
     * Lazy overload — evaluates [tx] at send time, not at registration time.
     * Use when [tx] lives in a scoped register that is written by an earlier exec
     * in the same sequence (e.g. inside [lockOnGoal]).
     */
    fun track(tx: () -> Double): Closure = Channels.send({ Request.Track(tx()) }, { actor.tx })

    /**
     * Tag lost. Sweeps in [lastKnownDirection] (±1.0) at [SEARCH_POWER].
     * Automatically reverses direction when a soft limit is hit.
     * Transitions to Locked/Tracking as soon as [lockOnGoal] sends a Track message.
     */
    fun search(lastKnownDirection: Double = 1.0): Closure =
        Channels.send({ Request.Search(lastKnownDirection) }, { actor.tx })

    /**
     * Lazy overload — evaluates [lastKnownDirection] at send time.
     */
    fun search(lastKnownDirection: () -> Double): Closure =
        Channels.send({ Request.Search(lastKnownDirection()) }, { actor.tx })

    /**
     * Drive the turret at a raw power value.
     * Keep power in a scoped variable and update it from an exec {} each tick.
     */
    fun drive(power: Double): Closure =
        Channels.send({ Request.Drive(power.coerceIn(-1.0, 1.0)) }, { actor.tx })

    /**
     * Lazy overload — evaluates [power] at send time.
     */
    fun drive(power: () -> Double): Closure =
        Channels.send({ Request.Drive(power().coerceIn(-1.0, 1.0)) }, { actor.tx })

    /**
     * Cut servo power immediately and enter Idle.
     */
    fun hold(): Closure = update(Request.Hold)

    /**
     * Reset the dead-reckoning estimate to 0.0 at the current physical position.
     * Call this at the start of each match after placing the turret at centre.
     * Also clears any in-progress search direction.
     */
    fun resetEstimate() {
        estimatedPosition = 0.0
        prevTx = 0.0
        ticker.reset()
        derivativeTicker.reset()
    }
}
