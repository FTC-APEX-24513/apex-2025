package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScope
import org.firstinspires.ftc.teamcode.subsystems.interfaces.TransferSubsystem as ITransferSubsystem
import software.amazon.lastmile.kotlin.inject.anvil.ContributesBinding
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn

@Inject
@SingleIn(HardwareScope::class)
@ContributesBinding(HardwareScope::class)
class TransferSubsystem(hardwareMap: HardwareMap) : ITransferSubsystem {
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
    
    override fun transfer() = Channels.send({ Command.Transfer }, { actor.tx })
    override fun reset() = Channels.send({ Command.Reset }, { actor.tx })
    override fun setPosition(position: Double) = Channels.send({ Command.SetPosition(position) }, { actor.tx })
    override fun setPosition(position: () -> Double) = Channels.send({ Command.SetPosition(position()) }, { actor.tx })
    
    companion object {
        private const val DEFAULT_POSITION = 0.2639
        private const val TRANSFER_POSITION = 0.8
    }
}
