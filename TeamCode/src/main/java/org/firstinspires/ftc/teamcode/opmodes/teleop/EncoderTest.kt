package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial

@Suppress("UNUSED")
val encoderTeleOp = Mercurial.teleop {
    val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    val servo = hardwareMap.get(CRServo::class.java, "spindexer");
    val encoder = hardwareMap.get(AnalogInput::class.java, "spindexerEncoder");

    var targetAngle = 0.0;
    var power = 0.0;

    waitForStart()

    schedule(
        loop({ inLoop }, exec {
            val currentAngle = currentAngle(encoder);
            telemetry.addData("currentAngle", currentAngle)
            telemetry.addData("targetAngle", targetAngle)
            val delta = shortestDelta(targetAngle, currentAngle)
            telemetry.addData("delta", delta)

        })
    )

    bindSpawn(
        risingEdge { gamepad1.dpad_up },
        exec {
            power += .05
        }
    )

    bindSpawn(
        risingEdge { gamepad1.dpad_down },
        exec {
            power -= .05
        }
    )

    bindSpawn(
        risingEdge { gamepad1.cross },
        exec {
            power = .0
        }
    )


}

fun currentAngle(encoder: AnalogInput): Double {
    return (encoder.voltage / 3.3) * 360.0
}

private fun shortestDelta(target: Double, current: Double): Double {
    var diff = target - current
    while (diff > 180) diff -= 360
    while (diff < -180) diff += 360
    return diff
}