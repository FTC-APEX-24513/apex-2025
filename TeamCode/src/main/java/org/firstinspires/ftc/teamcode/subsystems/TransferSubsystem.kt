package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareFactory
import org.firstinspires.ftc.teamcode.di.HardwareScope
import kotlin.math.abs

@Config
@Inject
@HardwareScope
class TransferSubsystem(factory: HardwareFactory) : Subsystem<TransferSubsystem.State, TransferSubsystem.Request>() {

    companion object {
        @JvmField var DOWN_POS = 0.0
        @JvmField var UP_POS = 0.35
        @JvmField var HOLD_TIME_SECONDS = 0.4
        @JvmField var INTER_TRANSFER_DELAY_SECONDS = 0.25
        @JvmField var FIRST_TRANSFER_DELAY_SECONDS = 0.6
    }

    private val servos = arrayOf(
        factory.getServo("transfer0"),
        factory.getServo("transfer1"),
        factory.getServo("transfer2")
    )

    init {
        for (i in servos.indices) {
            val pos = if (i == 2) UP_POS else DOWN_POS
            servos[i].position = pos
        }
    }

    sealed interface State {
        object Idle : State
        data class Active(val targetIndex: Int, val endTimestamp: Long) : State
        data class Sequencing(val targetIndex: Int, val queue: List<Int>, val endTimestamp: Long, val isFirstStep: Boolean) : State
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
                    // Start sequence, marking isFirstStep = true
                    State.Sequencing(list.first(), list.drop(1), now + holdNs, true)
                } else {
                    State.Idle
                }
            }
            is Request.Stop -> State.Idle
        }
    }

    override val behavior = { register: VarRegister<State> ->
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
                    updateHardware(-1) // Ensure hardware is down during the gap

                    if (now > state.endTimestamp) {
                        val holdNs = (HOLD_TIME_SECONDS * 1e9).toLong()
                        // Resume sequencing. isFirstStep is always false after the first delay.
                        register.set(State.Sequencing(state.nextIndex, state.queue, now + holdNs, false))
                    }
                }
            }
        })
    }

    private fun updateHardware(targetIndex: Int) {
        for (i in servos.indices) {
            val shouldBeUp = (i == targetIndex)

            val finalPos = if (i == 2) {
                if (shouldBeUp) DOWN_POS else UP_POS
            } else {
                if (shouldBeUp) UP_POS else DOWN_POS
            }

            if (abs(servos[i].position - finalPos) > 0.001) {
                servos[i].position = finalPos
            }
        }
    }

    fun trigger(index: Int): Closure = update(Request.Single(index))

    fun triggerSequence(indices: Array<Int>): Closure = update(Request.Sequence(indices))

    fun stop(): Closure = update(Request.Stop)
}