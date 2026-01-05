package org.firstinspires.ftc.teamcode.subsystems.interfaces

import dev.frozenmilk.dairy.mercurial.continuations.Closure

interface OuttakeSubsystem {
    fun spinUp(): Closure
    fun launch(): Closure
    fun stop(): Closure
    
    // Variable RPM control for intelligent shooting
    fun spinToRPM(targetRPM: Double): Closure
    fun getCurrentRPM(): Double
    fun isAtTargetSpeed(targetRPM: Double, tolerance: Double = 0.05): Boolean
}
