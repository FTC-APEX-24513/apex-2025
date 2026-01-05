package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareScoped

@Inject
@HardwareScoped
class IntakeSubsystem(hardwareMap: HardwareMap) {
    private val motor = hardwareMap.dcMotor.get("intake").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    
    enum class State {
        IDLE,
        COLLECTING,
        EJECTING
    }

    private val actor = Actors.actor<State, State>(
        { State.IDLE },
        { _, message -> message },
        { stateRegister ->
            match(stateRegister)
                .branch(
                    State.IDLE,
                    exec { motor.power = 0.0 }
                )
                .branch(
                    State.COLLECTING,
                    exec { motor.power = RobotConstants.INTAKE_COLLECT_POWER }
                )
                .branch(
                    State.EJECTING,
                    exec { motor.power = RobotConstants.INTAKE_EJECT_POWER }
                )
                .assertExhaustive()
        }
    )
    
    fun collect(): Closure = Channels.send({ State.COLLECTING }, { actor.tx })
    fun eject(): Closure = Channels.send({ State.EJECTING }, { actor.tx })
    fun stop(): Closure = Channels.send({ State.IDLE }, { actor.tx })
}
