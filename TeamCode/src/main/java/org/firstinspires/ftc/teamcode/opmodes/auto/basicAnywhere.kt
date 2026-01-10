package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create

/**
 * Basic Anywhere Autonomous
 *
 * Simple autonomous that strafes right for 0.2 seconds.
 * Can be used from any starting position for basic movement testing.
 */
@Suppress("UNUSED")
val basicAnywhere = Mercurial.autonomous {

    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    waitForStart()

    schedule(sequence(
        exec {
            container.follower.setTeleOpDrive(0.0, 0.5, 0.0, true)
        },
        wait(0.2),
        exec {
            container.follower.setTeleOpDrive(0.0, 0.0, 0.0, true)
        }
    ))

    dropToScheduler()
}
