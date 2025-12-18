package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Base interface for all robot subsystems.
 *
 * Subsystems should not access hardware in their constructor.
 * Instead, hardware initialization should happen in [init].
 */
interface Subsystem {
    /**
     * Initialize the subsystem with hardware from the HardwareMap.
     * Called once during OpMode initialization phase.
     *
     * @param hardwareMap The FTC hardware map to retrieve devices from
     */
    fun init(hardwareMap: HardwareMap)

    /**
     * Called periodically during the OpMode loop.
     * Use this for telemetry updates, control loops, state machines, etc.
     *
     * Default implementation does nothing.
     */
    fun periodic() {}
}
