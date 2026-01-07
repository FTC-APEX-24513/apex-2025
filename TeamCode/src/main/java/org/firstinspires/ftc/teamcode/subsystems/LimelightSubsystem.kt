package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.constants.Alliance
import org.firstinspires.ftc.teamcode.constants.ShootingConstants
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import org.firstinspires.ftc.teamcode.physics.ShootingCalculator

@Inject
@HardwareScoped
class LimelightSubsystem(hardwareMap: HardwareMap) : Subsystem() {
    private val limelight = hardwareMap.get(Limelight3A::class.java, "limelight").also {
        it.setPollRateHz(90)
        it.pipelineSwitch(0)
        it.start()
    }

    enum class Pipeline {
        APRILTAG
    }

    var pipeline: Pipeline = Pipeline.APRILTAG
        private set

    override fun periodic(): Closure = noop()

    // Commands
    fun useAprilTagPipeline(): Closure = exec {
        pipeline = Pipeline.APRILTAG
        limelight.pipelineSwitch(Pipeline.APRILTAG.ordinal)
    }

    // Getters
    fun getTx(): Double {
        val result = limelight.latestResult
        return if (result != null && result.isValid) result.tx else 0.0
    }

    fun getTy(): Double {
        val result = limelight.latestResult
        return if (result != null && result.isValid) result.ty else 0.0
    }

    fun hasTarget(): Boolean {
        val result = limelight.latestResult
        return result != null && result.isValid
    }

    fun updateRobotOrientation(yawDegrees: Double) {
        limelight.updateRobotOrientation(yawDegrees)
    }

    fun getBotPoseMT2(): Pose3D? {
        val result = limelight.latestResult
        if (result == null || !result.isValid) return null
        return result.botpose_MT2
    }

    fun getDistance2DToGoal(alliance: Alliance): Double? {
        val pose = getBotPoseMT2() ?: return null

        val robotX = pose.position.x
        val robotY = pose.position.y

        val goalX = if (alliance == Alliance.BLUE)
            ShootingConstants.BLUE_GOAL_X_INCHES * 0.0254
        else
            ShootingConstants.RED_GOAL_X_INCHES * 0.0254

        val goalY = if (alliance == Alliance.BLUE)
            ShootingConstants.BLUE_GOAL_Y_INCHES * 0.0254
        else
            ShootingConstants.RED_GOAL_Y_INCHES * 0.0254

        return ShootingCalculator.distance2D(robotX, robotY, goalX, goalY)
    }
}
