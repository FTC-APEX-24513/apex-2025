package edu.exeter.apex.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import edu.exeter.apex.ftc.teamcode.constants.ELLPipeline
import java.lang.annotation.Inherited


object Limelight : Subsystem {
    override var dependency: Dependency<*> =
        SingleAnnotation(KotlinSubsystem.Attach::class.java) and Subsystem.DEFAULT_DEPENDENCY

    private val limelight by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")
    }

    fun useAprilTagPipeline(): Lambda {
        return Lambda("apriltag")
            .addRequirements(Limelight)
            .setInit { limelight.pipelineSwitch(ELLPipeline.APRILTAG.ordinal) }
    }

//    fun useOtherPipeline() {
//        limelight!!.pipelineSwitch(LLPipeline.OTHER.ordinal)
//    }

    override fun preUserInitHook(opMode: Wrapper) {
        limelight.setPollRateHz(90)
        limelight.pipelineSwitch(ELLPipeline.APRILTAG.ordinal)
        limelight.start()
    }

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach
}