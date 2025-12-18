package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import software.amazon.lastmile.kotlin.inject.anvil.ContributesBinding
import software.amazon.lastmile.kotlin.inject.anvil.SingleIn
import kotlin.math.abs
import kotlin.math.max

@Inject
@SingleIn(HardwareScope::class)
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

    fun stop() = exec {
        frontLeft.power = 0.0
        frontRight.power = 0.0
        backLeft.power = 0.0
        backRight.power = 0.0
    }
}
