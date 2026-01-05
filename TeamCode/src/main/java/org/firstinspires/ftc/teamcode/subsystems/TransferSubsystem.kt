package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScoped

@Inject
@HardwareScoped
class TransferSubsystem(hardwareMap: HardwareMap) {
    private val servo = hardwareMap.get(Servo::class.java, "transfer")
    private var targetPosition = DEFAULT_POSITION
    
    enum class State {
        DEFAULT, TRANSFERRING
    }
    
    sealed class Command {
        object Transfer : Command()
        object Reset : Command()
        data class SetPosition(val position: Double) : Command()
    }
    
    private val actor = Actors.actor<State, Command>(
        { State.DEFAULT },
        { _, cmd ->
            // Update target position based on command
            when (cmd) {
                is Command.Transfer -> {
                    targetPosition = TRANSFER_POSITION
                    State.TRANSFERRING
                }
                is Command.Reset -> {
                    targetPosition = DEFAULT_POSITION
                    State.DEFAULT
                }
                is Command.SetPosition -> {
                    targetPosition = cmd.position
                    State.TRANSFERRING
                }
            }
        },
        { stateRegister ->
            match(stateRegister)
                .branch(
                    State.DEFAULT,
                    exec { servo.position = DEFAULT_POSITION }
                )
                .branch(
                    State.TRANSFERRING,
                    exec { servo.position = targetPosition }
                )
                .assertExhaustive()
        }
    )
    
    fun transfer(): Closure = Channels.send({ Command.Transfer }, { actor.tx })
    fun reset(): Closure = Channels.send({ Command.Reset }, { actor.tx })
    fun setPosition(position: Double): Closure = Channels.send({ Command.SetPosition(position) }, { actor.tx })
    fun setPosition(position: () -> Double): Closure = Channels.send({ Command.SetPosition(position()) }, { actor.tx })
    
    companion object {
        private const val DEFAULT_POSITION = 0.2639
        private const val TRANSFER_POSITION = 0.8
    }
}
