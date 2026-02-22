package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.matchType
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister
import me.tatarka.inject.annotations.Inject
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.di.HardwareFactory
import org.firstinspires.ftc.teamcode.di.HardwareScope
import org.firstinspires.ftc.teamcode.di.group
import org.firstinspires.ftc.teamcode.hardware.ServoGroup

@Config
@Inject
@HardwareScope
class TransferSubsystem(factory: HardwareFactory) : Subsystem<TransferSubsystem.State, TransferSubsystem.Request>() {

    companion object {
        @JvmField
        var HOLD_TIME_SECONDS = 0.4
        @JvmField
        var INTER_TRANSFER_DELAY_SECONDS = 0.25
        @JvmField
        var FIRST_TRANSFER_DELAY_SECONDS = 0.6
    }

    private val servos = (factory.group<Servo>("transfer0", "transfer1", "transfer2") as ServoGroup).also {
        it[0].direction = Servo.Direction.REVERSE
        it.scaleRange(-0.1, 0.2)
        it[1].direction = Servo.Direction.FORWARD
        it[1].scaleRange(-0.2, 0.2)
        it[2].direction = Servo.Direction.REVERSE
        it[2].scaleRange(-0.2, 0.2)
    }

    init {
        servos.position = 0.0
    }

    sealed interface State {
        object Idle : State
        data class Active(val targetIndex: Int, val endTimestamp: Long) : State
        data class Sequencing(
            val targetIndex: Int,
            val queue: List<Int>,
            val endTimestamp: Long,
            val isFirstStep: Boolean
        ) : State

        data class Delaying(val nextIndex: Int, val queue: List<Int>, val endTimestamp: Long) : State
    }

    sealed interface Request {
        data class Single(val index: Int) : Request
        data class Sequence(val indices: Array<Int>) : Request
        object Stop : Request
    }

    override val initialState = { State.Idle }

    override val transition = { _: State, msg: Request ->
        val now = System.nanoTime()
        val holdNs = (HOLD_TIME_SECONDS * 1e9).toLong()

        when (msg) {
            is Request.Single -> State.Active(msg.index, now + holdNs)

            is Request.Sequence -> {
                val list = msg.indices.toList()
                if (list.isNotEmpty()) {
                    State.Sequencing(list.first(), list.drop(1), now + holdNs, true)
                } else {
                    State.Idle
                }
            }

            is Request.Stop -> State.Idle
        }
    }

    override val behavior = { register: VarRegister<State> ->
        matchType({ register.get() })
            .branch<State.Idle>(
                exec { updateHardware(-1) }
            )
            .branch<State.Active> { stateReg ->
                sequence(
                    exec { updateHardware(stateReg.get().targetIndex) },
                    wait { System.nanoTime() >= stateReg.get().endTimestamp },
                    exec { register.set(State.Idle) }
                )
            }
            .branch<State.Sequencing> { stateReg ->
                sequence(
                    exec { updateHardware(stateReg.get().targetIndex) },
                    wait { System.nanoTime() >= stateReg.get().endTimestamp },
                    exec {
                        val s = stateReg.get()
                        if (s.queue.isNotEmpty()) {
                            val delaySeconds = if (s.isFirstStep) FIRST_TRANSFER_DELAY_SECONDS
                                               else INTER_TRANSFER_DELAY_SECONDS
                            val now = System.nanoTime()
                            register.set(State.Delaying(s.queue.first(), s.queue.drop(1), now + (delaySeconds * 1e9).toLong()))
                        } else {
                            register.set(State.Idle)
                        }
                    }
                )
            }
            .branch<State.Delaying> { stateReg ->
                sequence(
                    exec { updateHardware(-1) },
                    wait { System.nanoTime() >= stateReg.get().endTimestamp },
                    exec {
                        val s = stateReg.get()
                        val now = System.nanoTime()
                        register.set(State.Sequencing(s.nextIndex, s.queue, now + (HOLD_TIME_SECONDS * 1e9).toLong(), false))
                    }
                )
            }
            .assertExhaustive()
    }

    // ---------------------------------------------------------------------------
    // Legacy behavior — flat loop(exec{}) polling System.nanoTime() directly.
    // Kept as a fallback in case the match+wait version proves unresponsive.
    // To switch back: replace `override val behavior` above with this one.
    // ---------------------------------------------------------------------------
    @Suppress("unused")
    private val behaviorLegacy = { register: VarRegister<State> ->
        loop(exec {
            val state = register.get()
            val now = System.nanoTime()

            when (state) {
                is State.Idle -> {
                    updateHardware(-1)
                }

                is State.Active -> {
                    updateHardware(state.targetIndex)

                    if (now > state.endTimestamp) {
                        register.set(State.Idle)
                    }
                }

                is State.Sequencing -> {
                    updateHardware(state.targetIndex)

                    if (now > state.endTimestamp) {
                        if (state.queue.isNotEmpty()) {
                            val nextIndex = state.queue.first()
                            val nextQueue = state.queue.drop(1)

                            val delaySeconds = if (state.isFirstStep) {
                                FIRST_TRANSFER_DELAY_SECONDS
                            } else {
                                INTER_TRANSFER_DELAY_SECONDS
                            }

                            val delayNs = (delaySeconds * 1e9).toLong()

                            register.set(State.Delaying(nextIndex, nextQueue, now + delayNs))
                        } else {
                            register.set(State.Idle)
                        }
                    }
                }

                is State.Delaying -> {
                    updateHardware(-1)

                    if (now > state.endTimestamp) {
                        val holdNs = (HOLD_TIME_SECONDS * 1e9).toLong()
                        register.set(State.Sequencing(state.nextIndex, state.queue, now + holdNs, false))
                    }
                }
            }
        })
    }

    private fun updateHardware(targetIndex: Int) {
        for (i in 0 until servos.size) {
            servos[i].position = if (i == targetIndex) 1.0 else 0.0
        }
    }

    fun trigger(index: Int): Closure = update(Request.Single(index))

    /** Lazy overload — evaluates [index] at send time, not at registration time. */
    fun trigger(index: () -> Int): Closure = Channels.send({ Request.Single(index()) }, { actor.tx })

    fun triggerSequence(indices: Array<Int>): Closure = update(Request.Sequence(indices))

    fun stop(): Closure = update(Request.Stop)
}