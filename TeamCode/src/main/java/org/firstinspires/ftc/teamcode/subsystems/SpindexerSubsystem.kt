package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import kotlin.math.abs

@Inject
@HardwareScoped
class SpindexerSubsystem(hardwareMap: HardwareMap) {
    private val servo = hardwareMap.get(CRServo::class.java, "spindexer") as DcMotorSimple
    private var currentPower = 0.0
    private var targetPosition = 0

    private val encoder = (servo as DcMotorController).let {
        hardwareMap.get(DcMotor::class.java, "spindexer")
    }

    enum class State {
        IDLE, ADVANCING, KICKING, ROTATING_LEFT, ROTATING_RIGHT
    }

    sealed class Command {
        object Advance : Command()
        object Kick : Command()
        object Stop : Command()
        object RotateLeft : Command()
        object RotateRight : Command()
        data class SetPower(val power: Double) : Command()
    }

    private val actor = Actors.actor<State, Command>({ State.IDLE }, { _, cmd ->
        if (cmd is Command.SetPower) {
            currentPower = cmd.power
        }
        when (cmd) {
            is Command.Advance -> State.ADVANCING
            is Command.Kick -> State.KICKING
            is Command.Stop -> State.IDLE
            is Command.RotateLeft -> {
                encoder?.let {
                    targetPosition = it.currentPosition - RobotConstants.SPINDEXER_TICKS_PER_POSITION
                }
                State.ROTATING_LEFT
            }

            is Command.RotateRight -> {
                encoder?.let {
                    targetPosition = it.currentPosition + RobotConstants.SPINDEXER_TICKS_PER_POSITION
                }
                State.ROTATING_RIGHT
            }

            is Command.SetPower -> State.ADVANCING
        }
    }, { stateRegister ->
        var state by stateRegister

        match(stateRegister)
            .branch(State.IDLE, exec { servo.power = 0.0 })
            .branch(State.ADVANCING, exec { servo.power = currentPower })
            .branch(
                State.KICKING, sequence(
                    exec { servo.power = RobotConstants.SPINDEXER_KICK_POWER },
                    wait(RobotConstants.SPINDEXER_KICK_DURATION),
                    exec { state = State.IDLE })
            ).branch(
                State.ROTATING_LEFT, sequence(
                    exec { servo.power = -RobotConstants.SPINDEXER_ROTATION_POWER },
                    loop(
                        { abs(encoder.currentPosition - targetPosition) > RobotConstants.SPINDEXER_POSITION_TOLERANCE },
                        wait(0.01)
                    ),
                    exec { servo.power = 0.0; state = State.IDLE })
            ).branch(
                State.ROTATING_RIGHT, sequence(
                    exec { servo.power = RobotConstants.SPINDEXER_ROTATION_POWER },
                    loop(
                        { abs(encoder.currentPosition - targetPosition) > RobotConstants.SPINDEXER_POSITION_TOLERANCE },
                        wait(0.01)
                    ),
                    exec { servo.power = 0.0; state = State.IDLE })
            ).assertExhaustive()
    })

    fun advance(): Closure = Channels.send({ Command.Advance }, { actor.tx })
    fun kick(): Closure = Channels.send({ Command.Kick }, { actor.tx })
    fun stop(): Closure = Channels.send({ Command.Stop }, { actor.tx })
    fun rotateLeft(): Closure = Channels.send({ Command.RotateLeft }, { actor.tx })
    fun rotateRight(): Closure = Channels.send({ Command.RotateRight }, { actor.tx })
    fun setPower(power: Double): Closure = Channels.send({ Command.SetPower(power) }, { actor.tx })
    fun setPower(power: () -> Double): Closure = Channels.send({ Command.SetPower(power()) }, { actor.tx })
}
