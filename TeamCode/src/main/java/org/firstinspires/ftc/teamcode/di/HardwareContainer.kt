package org.firstinspires.ftc.teamcode.di

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Fiber
import dev.frozenmilk.dairy.mercurial.continuations.Scheduler
import me.tatarka.inject.annotations.Component
import me.tatarka.inject.annotations.Provides
import me.tatarka.inject.annotations.Scope
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.util.VoltageCompensation

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
 * ```kotlin
 * val teleOp = Mercurial.teleop {
 *     val container = HardwareContainer::class.create(hardwareMap)
 *     
 *     // Schedule periodic updates for all subsystems (runs sequentially each loop)
 *     schedule(container.periodicLoop())
 *     
 *     // Use subsystems
 *     bindSpawn(risingEdge { gamepad1.a }, container.intake.collect())
 *     
 *     dropToScheduler()
 * }
 * ```
 */
@Component
@HardwareScoped
abstract class HardwareContainer(@get:Provides val hardwareMap: HardwareMap, @get:Provides val scheduler: Scheduler) {
    
    @Provides
    @HardwareScoped
    fun provideFollower(hardwareMap: HardwareMap): Follower {
        return Constants.createFollower(hardwareMap)
    }

    @Provides
    @HardwareScoped
    fun provideVoltageSensor(hardwareMap: HardwareMap): VoltageSensor {
        // Get the voltage sensor from the hardwareMap
        // In FTC, the Control Hub has a built-in voltage sensor
        return hardwareMap.voltageSensor.iterator().next()
    }

    // Subsystems
    abstract val intake: IntakeSubsystem
    abstract val outtake: OuttakeSubsystem
    abstract val limelight: LimelightSubsystem
    abstract val spindexer: SpindexerSubsystem
    abstract val transfer: TransferSubsystem
    abstract val follower: Follower

    // Utilities
    abstract val voltageCompensation: VoltageCompensation
    
    /**
     * Returns a Closure that runs all subsystem periodics sequentially in a loop.
     * Schedule this once in your OpMode to enable automatic subsystem updates.
     * 
     * Subsystems are updated in a fixed, deterministic order:
     * follower -> intake -> outtake -> spindexer -> transfer -> limelight
     */
    fun startPeriodic(): Fiber = scheduler.schedule(loop({ true },
        sequence(
            exec { follower.update() },
            intake.periodic(),
            outtake.periodic(),
            spindexer.periodic(),
            transfer.periodic(),
            limelight.periodic()
        )
    ).intoContinuation())

    companion object
}
