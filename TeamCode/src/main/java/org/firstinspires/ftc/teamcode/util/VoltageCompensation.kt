package org.firstinspires.ftc.teamcode.util

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.VoltageSensor
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScope

@Configurable
@Inject
@HardwareScope
class VoltageCompensation(private val voltageSensor: VoltageSensor) {

    companion object {
        @JvmField
        var NOMINAL_VOLTAGE = 12.0

        @JvmField
        var ENABLE_COMPENSATION = true

        @JvmField
        var MAX_COMPENSATION_MULTIPLIER = 1.4
    }

    fun getVoltage(): Double = voltageSensor.voltage

    fun compensate(requestedPower: Double): Double {
        if (!ENABLE_COMPENSATION) return requestedPower

        val currentVoltage = getVoltage()
        if (currentVoltage <= 0.0) return requestedPower

        val compensationMultiplier = (NOMINAL_VOLTAGE / currentVoltage).coerceAtMost(MAX_COMPENSATION_MULTIPLIER)
        val compensatedPower = requestedPower * compensationMultiplier

        return compensatedPower.coerceIn(-1.0, 1.0)
    }

    fun getCompensationMultiplier(): Double {
        if (!ENABLE_COMPENSATION) return 1.0

        val currentVoltage = getVoltage()
        if (currentVoltage <= 0.0) return 1.0

        return (NOMINAL_VOLTAGE / currentVoltage).coerceAtMost(MAX_COMPENSATION_MULTIPLIER)
    }
}
