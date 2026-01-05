package org.firstinspires.ftc.teamcode.subsystems.interfaces

import dev.frozenmilk.dairy.mercurial.continuations.Closure

interface TransferSubsystem {
    fun transfer(): Closure
    fun reset(): Closure
    fun setPosition(position: Double): Closure
    fun setPosition(position: () -> Double): Closure
}
