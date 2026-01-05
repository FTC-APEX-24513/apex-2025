package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

@Inject
@HardwareScoped
class MecanumSubsystem(hardwareMap: HardwareMap) {
    private val frontLeft = hardwareMap.get(DcMotor::class.java, "leftFront").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        direction = DcMotorSimple.Direction.REVERSE
    }
    private val backLeft = hardwareMap.get(DcMotor::class.java, "leftRear").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        direction = DcMotorSimple.Direction.REVERSE
    }
    private val frontRight = hardwareMap.get(DcMotor::class.java, "rightFront").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        direction = DcMotorSimple.Direction.REVERSE
    }
    private val backRight = hardwareMap.get(DcMotor::class.java, "rightRear").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        direction = DcMotorSimple.Direction.FORWARD
    }
    
    // IMU for field-relative drive
    private val imu = hardwareMap.get(IMU::class.java, "imu").apply {
        initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            )
        )
    }

    /**
     * Drive the robot using mecanum kinematics.
     * Supports both robot-relative and field-relative control modes.
     *
     * @param axial Forward/backward power (-1 to 1)
     * @param lateral Left/right strafe power (-1 to 1)
     * @param yaw Rotation power (-1 to 1)
     * @param fieldRelative If true, controls are relative to field (driver perspective)
     */
    fun drive(axial: Double, lateral: Double, yaw: Double, fieldRelative: Boolean = false) {
        var axialInput = axial
        var lateralInput = lateral
        
        // Field-relative transformation if enabled
        if (fieldRelative) {
            val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            val cosHeading = cos(-heading)  // Negative because IMU yaw is opposite
            val sinHeading = sin(-heading)
            
            // Rotate input vector by heading
            axialInput = axial * cosHeading - lateral * sinHeading
            lateralInput = axial * sinHeading + lateral * cosHeading
        }
        
        // Mecanum kinematics
        var frontLeftPower = axialInput + lateralInput + yaw
        var frontRightPower = axialInput - lateralInput - yaw
        var backLeftPower = axialInput - lateralInput + yaw
        var backRightPower = axialInput + lateralInput - yaw

        // Normalize the values so no wheel power exceeds 100%
        var maxPower = max(abs(frontLeftPower), abs(frontRightPower))
        maxPower = max(maxPower, abs(backLeftPower))
        maxPower = max(maxPower, abs(backRightPower))

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower
            frontRightPower /= maxPower
            backLeftPower /= maxPower
            backRightPower /= maxPower
        }

        frontLeft.power = frontLeftPower
        frontRight.power = frontRightPower
        backLeft.power = backLeftPower
        backRight.power = backRightPower
    }

    fun stop(): Closure = exec {
        frontLeft.power = 0.0
        frontRight.power = 0.0
        backLeft.power = 0.0
        backRight.power = 0.0
    }
    
    /**
     * Reset IMU yaw to zero. Useful for setting current heading as "forward".
     */
    fun resetHeading() {
        imu.resetYaw()
    }
    
    fun getHeadingDegrees(): Double {
        return imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
    }
}
