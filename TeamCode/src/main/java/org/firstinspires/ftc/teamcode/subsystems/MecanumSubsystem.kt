package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.RobotScope
import software.amazon.lastmile.kotlin.inject.anvil.ContributesBinding
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn
import kotlin.math.abs
import kotlin.math.max

@Inject
@SingleIn(RobotScope::class)
@ContributesBinding(RobotScope::class, multibinding = true)
class MecanumSubsystem : Subsystem {
    private lateinit var frontLeft: DcMotor
    private lateinit var backLeft: DcMotor
    private lateinit var frontRight: DcMotor
    private lateinit var backRight: DcMotor

    override fun init(hardwareMap: HardwareMap) {
        frontLeft = hardwareMap.get(DcMotor::class.java, "leftFront").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            direction = DcMotorSimple.Direction.REVERSE
        }
        backLeft = hardwareMap.get(DcMotor::class.java, "leftRear").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            direction = DcMotorSimple.Direction.REVERSE
        }
        frontRight = hardwareMap.get(DcMotor::class.java, "rightFront").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            direction = DcMotorSimple.Direction.REVERSE
        }
        backRight = hardwareMap.get(DcMotor::class.java, "rightRear").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            direction = DcMotorSimple.Direction.FORWARD
        }
    }

    /**
     * Drive the robot using mecanum kinematics.
     *
     * @param axial Forward/backward power (-1 to 1)
     * @param lateral Left/right strafe power (-1 to 1)
     * @param yaw Rotation power (-1 to 1)
     */
    fun drive(axial: Double, lateral: Double, yaw: Double) {
        var frontLeftPower = axial + lateral + yaw
        var frontRightPower = axial - lateral - yaw
        var backLeftPower = axial - lateral + yaw
        var backRightPower = axial + lateral - yaw

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

    fun stop() {
        frontLeft.power = 0.0
        frontRight.power = 0.0
        backLeft.power = 0.0
        backRight.power = 0.0
    }

    override fun periodic() {
        // Optional: Add telemetry updates or control loops here
    }
}
