package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.RobotScope
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn

/**
 * Main robot class that holds all subsystems and manages their lifecycle.
 *
 * The [subsystems] set is automatically populated by kotlin-inject-anvil
 * with all classes that use @ContributesBinding with multibinding = true.
 *
 * Individual subsystems are also injected directly for easy access.
 */
@Inject
@SingleIn(RobotScope::class)
class Robot(
    private val subsystems: Set<Subsystem>,
    val mecanum: MecanumSubsystem,
    val intake: IntakeSubsystem,
    val outtake: OuttakeSubsystem,
    val limelight: LimelightSubsystem,
    val spindexer: SpindexerSubsystem,
    val transfer: TransferSubsystem
) {
    /**
     * Initialize all subsystems with the hardware map.
     * Call this in the OpMode init phase.
     *
     * @param hardwareMap The FTC hardware map
     */
    fun init(hardwareMap: HardwareMap) {
        subsystems.forEach { it.init(hardwareMap) }
    }

    /**
     * Call periodic() on all subsystems.
     * Call this in the OpMode loop.
     */
    fun periodic() {
        subsystems.forEach { it.periodic() }
    }
}
