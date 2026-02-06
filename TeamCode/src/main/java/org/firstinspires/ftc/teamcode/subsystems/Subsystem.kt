package org.firstinspires.ftc.teamcode.subsystems

import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister

/**
 * A base class for all subsystems using Mercurial Actors.
 */
abstract class Subsystem<S, M> {
    protected abstract val initialState: () -> S
    protected abstract val transition: (S, M) -> S
    protected abstract val behavior: (VarRegister<S>) -> Closure

    val actor by lazy {
        Actors.actor(
            initialState,
            transition,
            behavior
        )
    }

    /**
     * Helper to send a message to this subsystem's Actor.
     */
    protected fun update(message: M): Closure {
        return Channels.send({ message }, { actor.tx })
    }
}