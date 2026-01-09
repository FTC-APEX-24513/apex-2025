package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config


@Config
class RobotConstants {
    companion object {

        // ═══════════════════════════════════════════════════════
        // SHOOTING CONFIGURATION
        // ═══════════════════════════════════════════════════════

        @JvmField
        var SHOT_RECALC_THRESHOLD = 0.25  // meters (~10 inches - DECODE goal opening)

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
        // INTAKE CONFIGURATION
        // ═══════════════════════════════════════════════════════

        @JvmField
        var INTAKE_COLLECT_POWER = 0.9

        @JvmField
        var INTAKE_EJECT_POWER = -0.9

        @JvmField
        var INTAKE_TRIGGER_THRESHOLD = 0.01

        // ═══════════════════════════════════════════════════════
        // OUTTAKE / FLYWHEEL CONFIGURATION
        // ═══════════════════════════════════════════════════════

        @JvmField
        var OUTTAKE_TICKS_PER_REVOLUTION = 28.0

        @JvmField
        var OUTTAKE_RPM_PROPORTIONAL_GAIN = 0.0002

        @JvmField
        var OUTTAKE_DEFAULT_SPINUP_RPM = 3000.0

        @JvmField
        var OUTTAKE_LAUNCH_DURATION = 0.3

        // ═══════════════════════════════════════════════════════
        // TRANSFER CONFIGURATION
        // ═══════════════════════════════════════════════════════

        @JvmField
        var TRANSFER_DEFAULT_POSITION = 0.2639

        @JvmField
        var TRANSFER_TRANSFER_POSITION = -0.8
    }
}
