package org.firstinspires.ftc.teamcode.subsystems.interfaces

import dev.frozenmilk.dairy.mercurial.continuations.Closure
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.constants.Alliance

interface LimelightSubsystem {
    fun useAprilTagPipeline(): Closure
    fun getTx(): Double
    fun getTy(): Double
    fun hasTarget(): Boolean
    
    // MegaTag2 support
    fun updateRobotOrientation(yawDegrees: Double)
    fun getBotPoseMT2(): Pose3D?
    fun getDistance2DToGoal(alliance: Alliance): Double?
}
