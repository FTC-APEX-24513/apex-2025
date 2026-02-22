package org.firstinspires.ftc.teamcode.opmodes.test

import android.graphics.Color
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create

@Suppress("UNUSED")
val indexerTest = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val s00 = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor00")
    val s01 = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor01")
    val s10 = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor10")
    val s11 = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor11")
//    val s20 = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor20")
//    val s21 = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor21")

    val gain = 25.0f
    listOf(s00, s01, s10, s11 /*, s20, s21*/).forEach { it.gain = gain }

    fun getHsvString(s: NormalizedColorSensor): String {
        val c = s.normalizedColors
        val hsv = FloatArray(3)
        Color.RGBToHSV(
            (c.red * 255).toInt(),
            (c.green * 255).toInt(),
            (c.blue * 255).toInt(),
            hsv
        )
        return "H:${hsv[0].toInt()} S:${"%.2f".format(hsv[1])} A:${"%.2f".format(c.alpha)}"
    }

    waitForStart()

    schedule(
        loop({ inLoop }, exec {
            val inventory = container.indexer.inventory

            telemetry.addLine("=== SLOT 0 ===")
            telemetry.addData("DECISION", inventory[0])
            telemetry.addData("  Sens 00", getHsvString(s00))
            telemetry.addData("  Sens 01", getHsvString(s01))

            telemetry.addLine("\n=== SLOT 1 ===")
            telemetry.addData("DECISION", inventory[1])
            telemetry.addData("  Sens 10", getHsvString(s10))
            telemetry.addData("  Sens 11", getHsvString(s11))

//            telemetry.addLine("\n=== SLOT 2 (Top) ===")
//            telemetry.addData("DECISION", inventory[2])
//            telemetry.addData("  Sens 20", getHsvString(s20))
//            telemetry.addData("  Sens 21", getHsvString(s21))

            telemetry.update()
        })
    )

    dropToScheduler()
}