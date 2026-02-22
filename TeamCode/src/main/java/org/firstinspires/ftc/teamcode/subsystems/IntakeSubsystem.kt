package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareFactory
import org.firstinspires.ftc.teamcode.di.HardwareScope

@Config
@Inject
@HardwareScope
class IntakeSubsystem(factory: HardwareFactory) : Subsystem<IntakeSubsystem.State, IntakeSubsystem.State>() {

    private val servo = factory.getMotor("intake").apply {
        this.direction = DcMotorSimple.Direction.REVERSE
    }

    companion object {
        @JvmField var COLLECT_POWER = 0.85
        @JvmField var EJECT_POWER = -0.85
    }

    sealed interface State {
        object Idle : State
        object Collecting : State
        object Ejecting : State
    }

    override val initialState = { State.Idle }
    override val transition = { _: State, msg: State -> msg }

    override val behavior = { register: VarRegister<State> ->
        match { register.get() }
            .branch(State.Idle, exec {
                servo.power = 0.0
            })
            .branch(State.Collecting, loop(exec {
                servo.power = COLLECT_POWER
            }))
            .branch(State.Ejecting, loop(exec {
                servo.power = EJECT_POWER
            }))
            .assertExhaustive()
    }

    fun collect(): Closure = update(State.Collecting)
    fun eject(): Closure = update(State.Ejecting)
    fun stop(): Closure = update(State.Idle)
}