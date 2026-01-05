package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Actors
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareScope
import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSubsystem as IIntakeSubsystem
import software.amazon.lastmile.kotlin.inject.anvil.ContributesBinding
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn

@Inject
@SingleIn(HardwareScope::class)
@ContributesBinding(HardwareScope::class)
class IntakeSubsystem(hardwareMap: HardwareMap) : IIntakeSubsystem {
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
    
    override fun collect() = Channels.send({ State.COLLECTING }, { actor.tx })
    override fun eject() = Channels.send({ State.EJECTING }, { actor.tx })
    override fun stop() = Channels.send({ State.IDLE }, { actor.tx })
}
