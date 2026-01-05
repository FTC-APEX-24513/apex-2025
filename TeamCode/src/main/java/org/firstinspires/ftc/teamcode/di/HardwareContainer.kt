package org.firstinspires.ftc.teamcode.di

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.HardwareMap
import me.tatarka.inject.annotations.Component
import me.tatarka.inject.annotations.Provides
import me.tatarka.inject.annotations.Scope
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.*

/**
 * Scope annotation for hardware-related dependencies.
 * Ensures singletons are created once per HardwareContainer instance.
 */
@Scope
@Retention(AnnotationRetention.RUNTIME)
annotation class HardwareScoped

/**
 * Main Dependency Injection container for the robot.
 *
 * Usage in OpMode:
 * ```
 * val container = HardwareContainer.create(hardwareMap)
 * ```
 *
 * Uses plain kotlin-inject with @Inject annotations on subsystem classes.
 * This avoids classloader conflicts with Mercurial/Dairy's SlothClassLoader.
 */
@Component
@HardwareScoped
abstract class HardwareContainer(@get:Provides val hardwareMap: HardwareMap) {
    
    /**
     * Provider for Pedro Pathing Follower.
     * Creates and configures the Follower instance for odometry.
     */
    @Provides
    @HardwareScoped
    fun provideFollower(hardwareMap: HardwareMap): Follower {
        return Constants.createFollower(hardwareMap)
    }
    
    // Subsystems are automatically provided via @Inject constructors
    abstract val drive: MecanumSubsystem
    abstract val intake: IntakeSubsystem
    abstract val outtake: OuttakeSubsystem
    abstract val limelight: LimelightSubsystem
    abstract val spindexer: SpindexerSubsystem
    abstract val transfer: TransferSubsystem
    abstract val follower: Follower

    companion object
}
