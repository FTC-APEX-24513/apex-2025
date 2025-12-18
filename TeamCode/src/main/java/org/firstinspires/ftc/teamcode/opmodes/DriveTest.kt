package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create

@TeleOp(name = "drivetest")
class DriveTest : LinearOpMode() {
    private val runtime = ElapsedTime()

    override fun runOpMode() {
        // Create the DI component and get the hardware
        val container = HardwareContainer::class.create(hardwareMap)

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        runtime.reset()

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate
            val axial = -gamepad1.left_stick_y.toDouble()
            val lateral = gamepad1.left_stick_x.toDouble()
            val yaw = gamepad1.right_stick_x.toDouble()

            // Drive using the mecanum subsystem
            container.mecanum.drive(axial, lateral, yaw)

            // Show telemetry
            telemetry.addData("Status", "Run Time: $runtime")
            telemetry.update()
        }
    }
}