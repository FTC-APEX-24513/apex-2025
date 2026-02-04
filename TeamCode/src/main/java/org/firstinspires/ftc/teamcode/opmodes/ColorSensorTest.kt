package org.firstinspires.ftc.teamcode.opmodes

import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial

@Suppress("UNUSED")
val colorSensorTest = Mercurial.teleop {
    val cs00 = hardwareMap.colorSensor.get("colorSensor00")
    val cs01 = hardwareMap.colorSensor.get("colorSensor01")
    val cs10 = hardwareMap.colorSensor.get("colorSensor10")
    val cs11 = hardwareMap.colorSensor.get("colorSensor11")

    waitForStart()

    schedule(
        loop({ inLoop }, exec {
            // Display color sensor readings
            telemetry.addLine("=== COLOR SENSOR READINGS ===")
            telemetry.addData("CS00 - ARGB", cs00.argb())
            telemetry.addData("CS01 - ARGB", cs01.argb())
            telemetry.addData("CS10 - ARGB", cs10.argb())
            telemetry.addData("CS11 - ARGB", cs11.argb())
            telemetry.update()
        })
    )

    dropToScheduler()
}
