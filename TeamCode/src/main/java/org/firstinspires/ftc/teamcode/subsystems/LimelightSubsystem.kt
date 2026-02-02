package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.follower.Follower
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.di.HardwareScope

@Configurable
@Inject
@HardwareScope
class LimelightSubsystem(hardwareMap: HardwareMap, val follower: Follower) : Subsystem() {
    private val limelight = hardwareMap.get(Limelight3A::class.java, "limelight").also {
        it.setPollRateHz(90)
        it.pipelineSwitch(0)
        it.start()
    }

    companion object {
        const val METERS_PER_INCH = 0.0254
    }

    enum class Pipeline {
        APRILTAG
    }

    var pipeline: Pipeline = Pipeline.APRILTAG
        private set

    override fun periodic(): Closure = exec {
        limelight.updateRobotOrientation(Math.toDegrees(follower.pose.heading))
    }

    fun useAprilTagPipeline(): Closure = exec {
        pipeline = Pipeline.APRILTAG
        limelight.pipelineSwitch(Pipeline.APRILTAG.ordinal)
    }

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

    fun getPose(): Pose3D? {
        val result = limelight.latestResult
        if (result == null || !result.isValid) return null
        return result.botpose_MT2
    }
}
