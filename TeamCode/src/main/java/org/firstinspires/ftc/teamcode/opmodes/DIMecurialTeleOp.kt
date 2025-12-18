package org.firstinspires.ftc.teamcode.opmodes

import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create

@Suppress("UNUSED")
val diMecurialTeleOp = Mercurial.teleop {
    // Instantiate the DI component
    val container = HardwareContainer::class.create(hardwareMap)
    
    // Get the subsystem from the component
    val mecanum = container.mecanum

    var throttle = 1.0

    // Mecanum drive loop
    schedule(
        sequence(
            // Wait until the OpMode starts (play button pressed)
            wait { inLoop },
            loop(exec {
                val axial = -gamepad1.left_stick_y.toDouble() // Forward/Backward
                val lateral = gamepad1.left_stick_x.toDouble() // Strafe Left/Right
                val yaw = gamepad1.right_stick_x.toDouble()   // Turn Left/Right

                // Use the subsystem to drive with the calculated values
                mecanum.drive(axial * throttle, lateral * throttle, yaw * throttle)
            })
        )
    )

    // throttle controls
    bindSpawn(
        risingEdge { gamepad1.right_bumper },
        exec { throttle = 0.5 }
    )

    bindSpawn(
        // inverting the condition will convert our rising edge detector to a falling edge detector!
        risingEdge { !gamepad1.right_bumper },
        exec { throttle = 1.0 }
    )

    dropToScheduler()
}
