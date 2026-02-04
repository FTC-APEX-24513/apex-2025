package org.firstinspires.ftc.teamcode.di

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
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

@Scope
@Retention(AnnotationRetention.RUNTIME)
annotation class HardwareScope

@Component
@HardwareScope
abstract class HardwareContainer(@get:Provides val hardwareMap: HardwareMap, @get:Provides val scheduler: Scheduler) {

    @Provides
    @HardwareScope
    fun provideFollower(hardwareMap: HardwareMap): Follower {
        return Constants.createFollower(hardwareMap)
    }

    @Provides
    @HardwareScope
    fun provideVoltageSensor(hardwareMap: HardwareMap): VoltageSensor {
        return hardwareMap.voltageSensor.iterator().next()
    }

    abstract val intake: IntakeSubsystem
    abstract val outtake: OuttakeSubsystem
    abstract val limelight: LimelightSubsystem
    abstract val transfer: TransferSubsystem
    abstract val follower: Follower
    abstract val voltageCompensation: VoltageCompensation

    fun startPeriodic(): Fiber = scheduler.schedule(
        loop(
            { true }, sequence(
                exec { follower.update() },
                intake.periodic(),
                outtake.periodic(),
                transfer.periodic(),
                limelight.periodic()
            )
        ).intoContinuation()
    )

    companion object
}
