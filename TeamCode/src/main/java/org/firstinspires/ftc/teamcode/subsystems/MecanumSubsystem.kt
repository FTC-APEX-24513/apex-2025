package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.di.HardwareScoped
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.math.abs
import kotlin.math.max

@Inject
@HardwareScoped
class MecanumSubsystem(hardwareMap: HardwareMap) : Subsystem() {
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

    private val imu = hardwareMap.get(IMU::class.java, "imu").apply {
        initialize(IMU.Parameters(Constants.imuOrientation))
    }

    // Drive is called directly, no periodic needed
    override fun periodic(): Closure = noop()

    /**
     * Drive the robot using mecanum kinematics.
     */
    fun drive(axial: Double, lateral: Double, yaw: Double) {
        var frontLeftPower = axial + lateral + yaw
        var frontRightPower = axial - lateral - yaw
        var backLeftPower = axial - lateral + yaw
        var backRightPower = axial + lateral - yaw

        // Normalize
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

    fun resetHeading() {
        imu.resetYaw()
    }

    fun getHeadingDegrees(): Double {
        return imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
    }
}
