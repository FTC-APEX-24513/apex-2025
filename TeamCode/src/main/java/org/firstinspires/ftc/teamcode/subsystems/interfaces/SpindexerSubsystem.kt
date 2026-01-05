package org.firstinspires.ftc.teamcode.subsystems.interfaces

import dev.frozenmilk.dairy.mercurial.continuations.Closure

interface SpindexerSubsystem {
    fun advance(): Closure
    fun kick(): Closure
    fun stop(): Closure
    fun rotateLeft(): Closure
    fun rotateRight(): Closure
    fun setPower(power: Double): Closure
    fun setPower(power: () -> Double): Closure
}
