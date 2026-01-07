package org.firstinspires.ftc.teamcode.subsystems

import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop

/**
 * Base class for all robot subsystems.
 * 
 * Subsystems extend this class and override [periodic] to define
 * state-based hardware updates that run every loop iteration.
 * 
 * Example:
 * ```kotlin
 * class IntakeSubsystem(hardwareMap: HardwareMap) : Subsystem() {
 *     private val motor = hardwareMap.dcMotor.get("intake")
 *     var power = 0.0
 *     
 *     override fun periodic(): Closure = exec {
 *         motor.power = power
 *     }
 * }
 * ```
 */
abstract class Subsystem {
    /**
     * Closure that runs every loop iteration to update hardware.
     * Override this to define subsystem behavior.
     * 
     * Default implementation returns [noop] (does nothing).
     */
    open fun periodic(): Closure = noop()
}
