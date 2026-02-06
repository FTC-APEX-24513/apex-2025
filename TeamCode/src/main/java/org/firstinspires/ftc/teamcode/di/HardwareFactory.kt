package org.firstinspires.ftc.teamcode.di

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.util.VoltageCompensation
import org.firstinspires.ftc.teamcode.util.VCCRServo
import org.firstinspires.ftc.teamcode.util.VCMotor

@HardwareScope
@Inject
class HardwareFactory(
    private val hardwareMap: HardwareMap,
    private val voltageCompensation: VoltageCompensation
) {
    fun getMotor(name: String): DcMotorEx {
        val rawMotor = hardwareMap.get(DcMotorEx::class.java, name)
        return VCMotor(rawMotor, voltageCompensation)
    }

    fun getServo(name: String): Servo {
        return hardwareMap.get(Servo::class.java, name)
    }

    fun getCRServo(name: String): CRServo {
        val rawServo = hardwareMap.get(CRServo::class.java, name)
        return VCCRServo(rawServo, voltageCompensation)
    }

    fun getLimelight(name: String): Limelight3A {
        return hardwareMap.get(Limelight3A::class.java, name)
    }

    fun getPinpoint(name: String): GoBildaPinpointDriver {
        return hardwareMap.get(GoBildaPinpointDriver::class.java, name)
    }

    fun getColorSensor(name: String): NormalizedColorSensor {
        return hardwareMap.get(NormalizedColorSensor::class.java, name)
    }
}