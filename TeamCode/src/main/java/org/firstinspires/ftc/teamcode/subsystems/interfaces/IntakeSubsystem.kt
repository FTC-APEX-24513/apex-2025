package org.firstinspires.ftc.teamcode.subsystems.interfaces

import dev.frozenmilk.dairy.mercurial.continuations.Closure

interface IntakeSubsystem {
    fun collect(): Closure
    fun eject(): Closure
    fun stop(): Closure
}
