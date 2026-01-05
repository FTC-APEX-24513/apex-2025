package org.firstinspires.ftc.teamcode.constants

import com.bylazar.configurables.annotations.Configurable

/**
 * Central configuration for all robot tuning parameters.
 * Uses Bylazar annotations for live tuning via Driver Station.
 */
object RobotConstants {
    
    // ═══════════════════════════════════════════════════════
    // SPINDEXER CONFIGURATION
    // ═══════════════════════════════════════════════════════

    @JvmField
    var SPINDEXER_TICKS_PER_POSITION = 280

    @JvmField
    var SPINDEXER_ROTATION_POWER = 0.6

    @JvmField
    var SPINDEXER_POSITION_TOLERANCE = 10

    @JvmField
    var SPINDEXER_KICK_POWER = 1.0

    @JvmField
    var SPINDEXER_KICK_DURATION = 0.3  // seconds
    
    
    // ═══════════════════════════════════════════════════════
    // SHOOTING CONFIGURATION
    // ═══════════════════════════════════════════════════════

    @JvmField
    var MAX_FLYWHEEL_RPM = 6000.0

    @JvmField
    var FLYWHEEL_SPINDOWN_TIME = 3.0  // seconds

    @JvmField
    var SHOT_RECALC_THRESHOLD = 0.25  // meters (~10 inches - DECODE goal opening)

    @JvmField
    var RPM_TOLERANCE = 50.0  // RPM within target to consider "ready"
    
    
    // ═══════════════════════════════════════════════════════
    // ALIGNMENT CONFIGURATION
    // ═══════════════════════════════════════════════════════

    @JvmField
    var ALIGNMENT_TOLERANCE = 2.0  // degrees

    @JvmField
    var ALIGNMENT_KP = 0.03

    @JvmField
    var ALIGNMENT_KD = 0.01
    
    
    // ═══════════════════════════════════════════════════════
    // DRIVE CONFIGURATION
    // ═══════════════════════════════════════════════════════

    @JvmField
    var DRIVE_DEADZONE = 0.05

    @JvmField
    var FIELD_RELATIVE_ENABLED_DEFAULT = false
    
    
    // ═══════════════════════════════════════════════════════
    // INTAKE CONFIGURATION
    // ═══════════════════════════════════════════════════════

    @JvmField
    var INTAKE_COLLECT_POWER = 0.9

    @JvmField
    var INTAKE_EJECT_POWER = -0.9

    @JvmField
    var INTAKE_TRIGGER_THRESHOLD = 0.1
    
    
    // ═══════════════════════════════════════════════════════
    // SINGLE SHOT REALIGNMENT
    // ═══════════════════════════════════════════════════════

    @JvmField
    var ENABLE_SHOT_REALIGNMENT = true
    
    // Note: Uses SHOT_RECALC_THRESHOLD (0.25m / ~10 inches) to determine if robot moved
}
