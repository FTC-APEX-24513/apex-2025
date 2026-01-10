package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.VoltageSensor
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScoped

/**
 * Utility class for compensating motor power based on battery voltage.
 *
 * In FTC robotics, motor power is voltage-dependent. As the battery drains from 14V (full charge)
 * to 11V (low charge), motors will run slower at the same power setting. This class compensates
 * for voltage variations to maintain consistent motor performance across battery charge levels.
 *
 * Usage:
 * ```kotlin
 * val compensatedPower = voltageCompensation.compensate(requestedPower)
 * motor.power = compensatedPower
 * ```
 */
@Config
@Inject
@HardwareScoped
class VoltageCompensation(private val voltageSensor: VoltageSensor) {

    companion object {
        /**
         * Nominal battery voltage (V). This is the voltage level at which motor powers are tuned.
         * Typically 12V for FTC, but can be adjusted based on when you tuned your robot.
         */
        @JvmField var NOMINAL_VOLTAGE = 12.0

        /**
         * Enable/disable voltage compensation. Useful for debugging or comparing performance.
         */
        @JvmField var ENABLE_COMPENSATION = true

        /**
         * Maximum compensation multiplier to prevent excessive power demands.
         * If battery is very low (e.g., 9V), this caps the compensation to avoid overcurrent.
         */
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
     *
     * Example:
     * - Nominal voltage: 12V
     * - Current voltage: 11V (battery draining)
     * - Requested power: 0.5
     * - Compensated power: 0.5 * (12/11) = 0.545
     *
     * This ensures the motor gets the same effective power regardless of battery charge.
     *
     * @param requestedPower The power value you want (0.0 to 1.0, or -1.0 to 1.0 for bidirectional)
     * @return The voltage-compensated power value, clamped to [-1.0, 1.0]
     */
    fun compensate(requestedPower: Double): Double {
        if (!ENABLE_COMPENSATION) {
            return requestedPower
        }

        val currentVoltage = getVoltage()
        if (currentVoltage <= 0.0) {
            // Safety check: if voltage sensor fails, return uncompensated power
            return requestedPower
        }

        val compensationMultiplier = (NOMINAL_VOLTAGE / currentVoltage).coerceAtMost(MAX_COMPENSATION_MULTIPLIER)
        val compensatedPower = requestedPower * compensationMultiplier

        return compensatedPower.coerceIn(-1.0, 1.0)
    }

    /**
     * Returns the current compensation multiplier being applied.
     * Useful for telemetry and debugging.
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
