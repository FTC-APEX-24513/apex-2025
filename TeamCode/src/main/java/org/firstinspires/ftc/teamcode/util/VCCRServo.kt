package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.CRServo

class VCCRServo(
    private val delegate: CRServo,
    private val voltageCompensation: VoltageCompensation
) : CRServo by delegate {

    override fun setPower(power: Double) {
        delegate.power = voltageCompensation.compensate(power)
//        delegate.power = power
    }
}