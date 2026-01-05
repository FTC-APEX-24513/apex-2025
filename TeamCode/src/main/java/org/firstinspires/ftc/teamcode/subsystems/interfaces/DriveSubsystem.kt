package org.firstinspires.ftc.teamcode.subsystems.interfaces

import dev.frozenmilk.dairy.mercurial.continuations.Closure

interface DriveSubsystem {
    fun drive(axial: Double, lateral: Double, yaw: Double, fieldRelative: Boolean = false)
    fun stop(): Closure
    fun getHeadingDegrees(): Double
}
