package org.firstinspires.ftc.teamcode.opmodes.test

import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create

/**
 * Test OpMode for the Transfer Subsystem.
 *
 * Controls
 * --------
 *   D-Pad Up       — Raise the currently selected transfer servo.
 *   D-Pad Down     — Lower all transfer servos (stop).
 *   Triangle       — Cycle selected index: 0 → 1 → 2 → 0 …
 *   Cross          — Trigger the currently selected index (single shot).
 *   Circle         — Sequence all three transfers in order (0, 1, 2).
 *
 * Telemetry
 * ---------
 *   Selected index is always shown.
 */
@Suppress("UNUSED")
val transferTest = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val transfer = container.transfer

    var selected = 0

    waitForStart()

    // D-Pad Up — raise the currently selected servo.
    // Lazy lambda reads `selected` at press time, not at registration time.
    bindSpawn(risingEdge { gamepad1.dpad_up }, transfer.trigger { selected })

    // D-Pad Down — lower all servos.
    bindSpawn(risingEdge { gamepad1.dpad_down }, transfer.stop())

    // Triangle — cycle selected index 0 → 1 → 2 → 0.
    bindSpawn(risingEdge { gamepad1.triangle }, exec { selected = (selected + 1) % 3 })

    // Cross — trigger the currently selected index.
    bindSpawn(risingEdge { gamepad1.cross }, transfer.trigger { selected })

    // Circle — sequence all three.
    bindSpawn(risingEdge { gamepad1.circle }, transfer.triggerSequence(arrayOf(0, 1, 2)))

    // Telemetry loop.
    schedule(loop({ inLoop }, exec {
        telemetry.addLine("=== TRANSFER TEST ===")
        telemetry.addData("Selected index", selected)
        telemetry.addLine("")
        telemetry.addLine("D-Pad Up   = raise selected")
        telemetry.addLine("D-Pad Down = lower all")
        telemetry.addLine("Triangle   = cycle index (0 -> 1 -> 2 -> 0)")
        telemetry.addLine("Cross      = trigger selected")
        telemetry.addLine("Circle     = sequence all (0, 1, 2)")
        telemetry.update()
    }))

    dropToScheduler()
}
