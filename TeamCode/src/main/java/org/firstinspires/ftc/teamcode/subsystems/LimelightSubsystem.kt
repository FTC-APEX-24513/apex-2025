package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.di.HardwareFactory
import org.firstinspires.ftc.teamcode.di.HardwareScope

@Config
@Inject
@HardwareScope
class LimelightSubsystem(factory: HardwareFactory) : Subsystem<LimelightSubsystem.Pipeline, LimelightSubsystem.Pipeline>() {

    private val limelight = factory.getLimelight("limelight").also {
        it.setPollRateHz(90)
        it.start()
    }

    private val pinpoint = factory.getPinpoint("pinpoint")

    companion object {
        const val METERS_PER_INCH = 0.0254

        @JvmField var BLUE_GOAL_X_INCHES = 144.0
        @JvmField var BLUE_GOAL_Y_INCHES = 72.0
    }

    enum class Pipeline {
        APRILTAG
    }

    override val initialState = { Pipeline.APRILTAG }

    override val transition = { _: Pipeline, newPipeline: Pipeline ->
        limelight.pipelineSwitch(newPipeline.ordinal)
        newPipeline
    }

    override val behavior = { register: VarRegister<Pipeline> ->
        match { register.get() }
            .branch(Pipeline.APRILTAG, loop(exec {
                pinpoint.update()
                val heading = pinpoint.getHeading(AngleUnit.DEGREES)
                limelight.updateRobotOrientation(heading)
            }))
            .assertExhaustive()
    }

    fun useAprilTagPipeline(): Closure = update(Pipeline.APRILTAG)

    fun getBotPoseMT2(): Pose3D? {
        val result = limelight.latestResult
        if (result == null || !result.isValid) return null
        return result.botpose_MT2
    }

    /**
     * Get the horizontal offset to the target (tx).
     * Negative = target is left of crosshair, Positive = right.
     * @return tx value in degrees, or null if no valid target
     */
    fun getTargetTx(): Double? {
        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null
        return result.tx
    }

    /**
     * Get the vertical offset to the target (ty).
     * Negative = target is below crosshair, Positive = above.
     * @return ty value in degrees, or null if no valid target
     */
    fun getTargetTy(): Double? {
        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null
        return result.ty
    }

    /**
     * Check if the Limelight currently has a valid target.
     */
    fun hasTarget(): Boolean {
        val result = limelight.latestResult ?: return false
        return result.isValid
    }
}