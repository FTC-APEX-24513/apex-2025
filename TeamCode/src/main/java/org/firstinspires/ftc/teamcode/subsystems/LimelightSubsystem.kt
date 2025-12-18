package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.constants.EPipeline
import org.firstinspires.ftc.teamcode.di.RobotScope
import software.amazon.lastmile.kotlin.inject.anvil.ContributesBinding
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn

@Inject
@SingleIn(RobotScope::class)
@ContributesBinding(RobotScope::class, multibinding = true)
class LimelightSubsystem : Subsystem {
    private lateinit var limelight: Limelight3A

    override fun init(hardwareMap: HardwareMap) {
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight").also {
            it.setPollRateHz(90)
            it.pipelineSwitch(EPipeline.APRILTAG.ordinal)
            it.start()
        }
    }

    fun useAprilTagPipeline() {
        limelight.pipelineSwitch(EPipeline.APRILTAG.ordinal)
    }

    override fun periodic() {
        // Optional: Process vision results, update pose estimation
    }
}
