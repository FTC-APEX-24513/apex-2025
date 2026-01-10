package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create

@Suppress("UNUSED")
val limelightTestTeleOp = Mercurial.teleop {
    val telemetryA = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val loopTimer = ElapsedTime()

    waitForStart()
    container.follower.startTeleopDrive()

    schedule(
        loop({ inLoop }, exec {
            val loopMs = loopTimer.milliseconds()
            loopTimer.reset()

            // 1. Drive Control (Pedro Pathing TeleOp)
            val axial = -gamepad1.left_stick_y.toDouble()
            val lateral =
                -gamepad1.left_stick_x.toDouble() // Standard Pedro: Left is +Y, so Right stick is -Y? Check if inverted.
            // Usually: 
            // Fwd: -StickY
            // Strafe Right: -StickX (if Left is positive Y)
            // Turn Right: -StickRX (if Left is positive Heading)

            val yaw = -gamepad1.right_stick_x.toDouble()

            container.follower.setTeleOpDrive(axial, lateral, yaw, true)

            // 2. Update Limelight with IMU Heading (Critical for MT2)
            // Pedro uses Radians. Limelight uses Degrees.
            val headingDeg = Math.toDegrees(container.follower.pose.heading)
            container.limelight.updateRobotOrientation(headingDeg)

            // 3. Get Data
            val mt2Pose = container.limelight.getBotPoseMT2()
            val hasTarget = container.limelight.hasTarget()

            // 4. Telemetry
            telemetryA.addLine("=== LIMELIGHT MT2 TEST ===")
            telemetryA.addData("Loop", "%.1f ms", loopMs)

            telemetryA.addLine("\n--- INPUTS ---")
            telemetryA.addData("Pedro Heading", "%.2f deg", headingDeg)
            telemetryA.addData("Pedro Pose", container.follower.pose.toString())

            telemetryA.addLine("\n--- VISION RESULTS ---")
            telemetryA.addData("Has Target", hasTarget)
            telemetryA.addData("Tx", "%.2f", container.limelight.getTx())
            telemetryA.addData("Ty", "%.2f", container.limelight.getTy())

            if (mt2Pose != null) {
                val pos = mt2Pose.position
                val orient = mt2Pose.orientation

                // Convert to inches for readability (FTC standard)
                val xInch = pos.toUnit(DistanceUnit.INCH).x
                val yInch = pos.toUnit(DistanceUnit.INCH).y
                val zInch = pos.toUnit(DistanceUnit.INCH).z

                telemetryA.addLine("\n--- MT2 POSE (Field Centric) ---")
                telemetryA.addData("X", "%.2f in", xInch)
                telemetryA.addData("Y", "%.2f in", yInch)
                telemetryA.addData("Z", "%.2f in", zInch)
                telemetryA.addData("Orient", mt2Pose.orientation.toString())
            } else {
                telemetryA.addLine("\n--- NO POSE ---")
                telemetryA.addLine("Robot not localized")
            }

            telemetryA.addLine("\n--- CONTROLS ---")
            telemetryA.addLine("LS: Drive | RS: Turn | RB: Slow")
            // telemetryA.addLine("Y (Triangle): Reset IMU Heading")

            telemetryA.update()
        })
    )

    // Bind IMU Reset (Attempting to find correct method)
    /*
    bindSpawn(
        risingEdge { gamepad1.triangle },
        exec {
             // container.follower.poseUpdater.resetIMU() // Uncomment if available
        }
    )
    */
}