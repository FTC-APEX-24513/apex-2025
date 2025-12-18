package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.constants.EPipeline
import org.firstinspires.ftc.teamcode.di.HardwareScope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import software.amazon.lastmile.kotlin.inject.anvil.ContributesBinding
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn

@Inject
@SingleIn(HardwareScope::class)
class LimelightSubsystem(hardwareMap: HardwareMap) {
    private val limelight = hardwareMap.get(Limelight3A::class.java, "limelight").also {
            it.setPollRateHz(90)
            it.pipelineSwitch(EPipeline.APRILTAG.ordinal)
            it.start()
        }

    fun useAprilTagPipeline() = exec {
        limelight.pipelineSwitch(EPipeline.APRILTAG.ordinal)
    }
}
