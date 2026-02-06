package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx

/**
 * A wrapper around DcMotorEx that automatically applies voltage compensation
 * whenever setPower() is called.
 */
class VCMotor(
    private val delegate: DcMotorEx,
    private val voltageCompensation: VoltageCompensation
) : DcMotorEx by delegate {
    override fun setPower(power: Double) {
        delegate.power = voltageCompensation.compensate(power)
    }
}