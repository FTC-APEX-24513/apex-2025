package org.firstinspires.ftc.teamcode.di

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.util.VCCRServo
import org.firstinspires.ftc.teamcode.util.VCMotor
import org.firstinspires.ftc.teamcode.util.VoltageCompensation

@HardwareScope
@Inject
class HardwareFactory(
    private val hardwareMap: HardwareMap,
    private val voltageCompensation: VoltageCompensation
) {
    fun getMotor(name: String): DcMotorEx =
        VCMotor(hardwareMap.get(DcMotorEx::class.java, name), voltageCompensation)

    fun getServo(name: String): Servo =
        hardwareMap.get(Servo::class.java, name)

    fun getCRServo(name: String): CRServo =
        VCCRServo(hardwareMap.get(CRServo::class.java, name), voltageCompensation)

    fun getLimelight(name: String): Limelight3A =
        hardwareMap.get(Limelight3A::class.java, name)

    fun getPinpoint(name: String): GoBildaPinpointDriver =
        hardwareMap.get(GoBildaPinpointDriver::class.java, name)

    fun getColorSensor(name: String): NormalizedColorSensor =
        hardwareMap.get(NormalizedColorSensor::class.java, name)

    fun getAnalogInput(name: String): AnalogInput =
        hardwareMap.get(AnalogInput::class.java, name)
}
