package org.firstinspires.ftc.teamcode.subsystems

import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop

abstract class Subsystem {
    open fun periodic(): Closure = noop()
}
