package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.VoltageSensor
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScope

/**
 * Utility class for compensating motor power based on battery voltage.
 */
@Config
@Inject
@HardwareScope
class VoltageCompensation(private val voltageSensor: VoltageSensor) {

    companion object {
        @JvmField var NOMINAL_VOLTAGE = 12.0
        @JvmField var ENABLE_COMPENSATION = false
        @JvmField var MAX_COMPENSATION_MULTIPLIER = 1.4
    }

    /**
     * Gets the current battery voltage in volts.
     */
    fun getVoltage(): Double = voltageSensor.voltage

    /**
     * Compensates a motor power value for the current battery voltage.
     *
     * Formula: compensatedPower = requestedPower * (nominalVoltage / currentVoltage)
     * @param requestedPower The power value you want (0.0 to 1.0, or -1.0 to 1.0 for bidirectional)
     * @return The voltage-compensated power value, clamped to [-1.0, 1.0]
     */
    fun compensate(requestedPower: Double): Double {
        if (!ENABLE_COMPENSATION) {
            return requestedPower
        }

        val currentVoltage = getVoltage()
        if (currentVoltage <= 0.0) {
            return requestedPower
        }

        val compensationMultiplier = (NOMINAL_VOLTAGE / currentVoltage).coerceAtMost(MAX_COMPENSATION_MULTIPLIER)
        val compensatedPower = requestedPower * compensationMultiplier

        return compensatedPower.coerceIn(-1.0, 1.0)
    }

    /**
     * Returns the current compensation multiplier being applied.
     *
     * @return The multiplier (e.g., 1.09 means 9% power boost to compensate for voltage drop)
     */
    fun getCompensationMultiplier(): Double {
        if (!ENABLE_COMPENSATION) {
            return 1.0
        }

        val currentVoltage = getVoltage()
        if (currentVoltage <= 0.0) {
            return 1.0
        }

        return (NOMINAL_VOLTAGE / currentVoltage).coerceAtMost(MAX_COMPENSATION_MULTIPLIER)
    }
}
