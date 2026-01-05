package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
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
import org.firstinspires.ftc.teamcode.constants.ShootingConstants
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import kotlin.math.abs

@Inject
@HardwareScoped
class OuttakeSubsystem(hardwareMap: HardwareMap) {
    // Cache as DcMotorEx for velocity reading - avoids repeated casts
    private val motor = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx::class.java, "flywheel").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    
    // Track target RPM as class variable since states are enums
    private var targetRPM: Double = 0.0
    
    enum class State {
        OFF,
        SPINNING_UP,
        READY,
        LAUNCHING
    }
    
    // Sealed class for commands
    sealed class Command {
        object Stop : Command()
        object SpinUpFixed : Command()  // Original fixed-speed spinup
        data class SpinToRPM(val targetRPM: Double) : Command()
        object Launch : Command()
    }

    private val actor = Actors.actor<State, Command>(
        { State.OFF },
        { state, command -> 
            // Update target RPM based on command
            when (command) {
                is Command.Stop -> {
                    targetRPM = 0.0
                    State.OFF
                }
                is Command.SpinUpFixed -> {
                    targetRPM = 3000.0
                    State.SPINNING_UP
                }
                is Command.SpinToRPM -> {
                    targetRPM = command.targetRPM
                    State.SPINNING_UP
                }
                is Command.Launch -> State.LAUNCHING
            }
        },
        { stateRegister ->
            var state by stateRegister
            
            match(stateRegister)
                .branch(
                    State.OFF,
                    exec { motor.power = 0.0 }
                )
                .branch(
                    State.SPINNING_UP,
                    sequence(
                        loop(
                            { !isAtTargetSpeed(targetRPM) },
                            exec {
                                val currentRPM = getCurrentRPM()
                                val power = calculatePowerForRPM(targetRPM, currentRPM)
                                motor.power = power
                            }
                        ),
                        exec { state = State.READY }
                    )
                )
                .branch(
                    State.READY,
                    exec {
                        val currentRPM = getCurrentRPM()
                        val power = calculatePowerForRPM(targetRPM, currentRPM)
                        motor.power = power
                    }
                )
                .branch(
                    State.LAUNCHING,
                    sequence(
                        exec { motor.power = 1.0 },
                        wait(0.3),
                        exec { state = State.OFF }  // Stop after launch
                    )
                )
                .assertExhaustive()
        }
    )
    
    /**
     * Get current flywheel RPM from motor encoder.
     * Uses getVelocity() which returns encoder ticks per second.
     * Optimized: motor is already cached as DcMotorEx, no cast needed.
     */
    fun getCurrentRPM(): Double {
        val ticksPerSecond = motor.velocity  // Direct property access, no cast
        val revolutionsPerSecond = ticksPerSecond / ShootingConstants.TICKS_PER_REVOLUTION
        return revolutionsPerSecond * 60.0
    }
    
    /**
     * Check if flywheel is at target speed within tolerance.
     */
    fun isAtTargetSpeed(targetRPM: Double, tolerance: Double = 0.05): Boolean {
        val currentRPM = getCurrentRPM()
        val error = abs(targetRPM - currentRPM) / targetRPM.coerceAtLeast(1.0)
        return error < tolerance
    }
    
    /**
     * Calculate motor power based on RPM error (simple proportional control).
     */
    private fun calculatePowerForRPM(targetRPM: Double, currentRPM: Double): Double {
        val error = targetRPM - currentRPM
        val power = ShootingConstants.RPM_PROPORTIONAL_GAIN * error
        return power.coerceIn(0.0, 1.0)
    }
    
    fun spinUp(): Closure = Channels.send({ Command.SpinUpFixed }, { actor.tx })
    fun launch(): Closure = Channels.send({ Command.Launch }, { actor.tx })
    fun stop(): Closure = Channels.send({ Command.Stop }, { actor.tx })
    fun spinToRPM(targetRPM: Double): Closure = Channels.send({ Command.SpinToRPM(targetRPM) }, { actor.tx })
}
