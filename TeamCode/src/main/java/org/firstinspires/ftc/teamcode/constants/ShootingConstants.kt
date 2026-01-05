package org.firstinspires.ftc.teamcode.constants

/**
 * Physical constants and tunable parameters for the intelligent shooting system.
 * All values should be measured and calibrated during testing.
 */
object ShootingConstants {
    // ========== Robot Physical Parameters ==========
    const val LAUNCH_HEIGHT_METERS = 0.2255         // 225.5mm from ground to flywheel
    const val LAUNCH_ANGLE_RADIANS = 0.0            // Horizontal launch

    // ========== Flywheel Physical Properties ==========
    const val FLYWHEEL_RADIUS_METERS = 0.060        // 60mm radius (120mm diameter) - MEASURE THIS
    
    // Hollow cylinder flywheel properties (MEASURE AND TUNE)
    const val FLYWHEEL_MASS_KG = 0.150              // TODO: Measure flywheel mass with scale
    const val FLYWHEEL_INNER_RADIUS_METERS = 0.055  // TODO: Measure inner radius (if applicable)
    
    // Moment of inertia for hollow cylinder: I = 0.5 * m * (r_outer² + r_inner²)
    // For thin-walled hollow cylinder (r_inner ≈ r_outer): I ≈ m * r²
    val FLYWHEEL_MOMENT_OF_INERTIA = 0.5 * FLYWHEEL_MASS_KG * 
        (FLYWHEEL_RADIUS_METERS * FLYWHEEL_RADIUS_METERS + 
         FLYWHEEL_INNER_RADIUS_METERS * FLYWHEEL_INNER_RADIUS_METERS)

    // ========== Ball Properties (from DECODE game manual) ==========
    const val BALL_DIAMETER_NOM_METERS = 0.1245     // 4.9" nominal diameter
    const val BALL_MASS_KG = 0.065                  // ~65g (from game manual)

    // Ball-Flywheel Contact Parameters (MEASURE AND TUNE)
    const val BALL_CONTACT_ARC_DEGREES = 45.0       // TODO: Measure contact arc in degrees
                                                     // Tip: Mark flywheel, hold ball against it, measure arc
                                                     // Typical range: 30-60 degrees

    // ========== GOAL Specifications (from game manual section 9.7) ==========
    const val GOAL_TOP_LIP_HEIGHT_METERS = 0.9845   // 38.75" from tiles
    const val GOAL_OPENING_WIDTH_METERS = 0.673     // 26.5"
    const val GOAL_OPENING_DEPTH_METERS = 0.465     // 18.3"

    // ========== Field Coordinates (Pedro Pathing uses inches) ==========
    // Field is 144" x 144" (12 tiles x 12 tiles)

    // Blue GOAL position (far end, centered)
    const val BLUE_GOAL_X_INCHES = 144.0
    const val BLUE_GOAL_Y_INCHES = 72.0

    // Red GOAL position (near end, centered)
    const val RED_GOAL_X_INCHES = 0.0
    const val RED_GOAL_Y_INCHES = 72.0

    // ========== Shooting Strategy Parameters ==========
    // Distance limits
    const val MAX_SHOT_DISTANCE_METERS = 3.5
    const val MIN_SHOT_DISTANCE_METERS = 0.3

    // ========== Flywheel Control Parameters ==========
    // Energy transfer efficiency from flywheel to ball
    const val FLYWHEEL_TO_BALL_EFFICIENCY = 0.87    // TODO: Calibrate with chronograph

    // RPM control tolerance
    const val TARGET_VELOCITY_TOLERANCE = 0.05      // ±5% RPM tolerance

    // Flywheel RPM limits
    const val MAX_FLYWHEEL_RPM = 5000.0             // Hardware limit
    const val MIN_STABLE_RPM = 500.0                // Minimum for consistent shooting

    // ========== Motor Configuration ==========
    // CRITICAL: This must be measured for your specific motor!
    // Run motor at known speed and count encoder ticks to calculate.
    const val TICKS_PER_REVOLUTION = 28.0           // PLACEHOLDER - REV HD Hex Motor default

    // Common motor values for reference:
    // - REV HD Hex Motor: 28 ticks/rev
    // - NeveRest 40: 1120 ticks/rev
    // - NeveRest 20: 560 ticks/rev
    // - REV Core Hex Motor: 4 ticks/rev

    // ========== Motor Re-acceleration Parameters ==========
    const val MOTOR_FREE_SPEED_RPM = 6000.0         // TODO: Check motor specs (e.g., NeveRest, REV HD Hex)
    const val MOTOR_STALL_TORQUE_NM = 0.170         // TODO: Check motor specs (Newton-meters)
    const val MOTOR_GEAR_RATIO = 1.0                // TODO: Set gear ratio (1.0 = direct drive)

    // Effective acceleration rate accounting for motor torque limits
    // angular_acceleration = torque / moment_of_inertia
    // rpm_acceleration_rate = (torque / I) * (60 / 2π) * gear_ratio
    val MOTOR_ACCELERATION_RATE_RPM_PER_SEC = 
        (MOTOR_STALL_TORQUE_NM / FLYWHEEL_MOMENT_OF_INERTIA) * (60.0 / (2.0 * kotlin.math.PI)) * MOTOR_GEAR_RATIO

    // ========== Recovery Time Parameters ==========
    const val MIN_RECOVERY_TIME_SECONDS = 0.15      // Minimum physical time (safety margin)
    const val RECOVERY_TIME_SAFETY_FACTOR = 1.2     // Add 20% safety margin to calculated time

    // ========== PID Control ==========
    // Proportional gain for RPM control
    const val RPM_PROPORTIONAL_GAIN = 0.0002        // TODO: Tune for your motor
    // Start with 0.0002, increase if too slow, decrease if oscillating

    // ========== Default RPM (when no AprilTag visible) ==========
    const val DEFAULT_SHOOTING_RPM = 3500.0         // Safe mid-range RPM for unknown distance

    // ========== AprilTag IDs (from game manual) ==========
    const val BLUE_GOAL_APRILTAG_ID = 20
    const val RED_GOAL_APRILTAG_ID = 24
}

/**
 * HOW TO MEASURE FLYWHEEL PROPERTIES:
 * 
 * 1. FLYWHEEL_MASS_KG:
 *    - Remove flywheel from robot
 *    - Weigh on digital scale (grams)
 *    - Convert to kg: weight_grams / 1000
 * 
 * 2. FLYWHEEL_RADIUS_METERS:
 *    - Measure outer diameter with calipers (mm)
 *    - Divide by 2, convert to meters: diameter_mm / 2000
 * 
 * 3. FLYWHEEL_INNER_RADIUS_METERS (if hollow):
 *    - Measure inner diameter (mm)
 *    - Convert: inner_diameter_mm / 2000
 *    - If thin-walled, set close to outer radius
 * 
 * 4. BALL_CONTACT_ARC_DEGREES:
 *    - Place ball against flywheel
 *    - Mark start and end of contact area on flywheel
 *    - Measure arc angle with protractor or estimate from arc length
 *    - Typical range: 30-60 degrees
 * 
 * 5. MOTOR SPECS:
 *    - Check motor datasheet for your specific motor
 *    - Common FTC motors:
 *      - REV HD Hex: 6000 RPM free speed, 0.17 Nm stall torque
 *      - NeveRest 40: 160 RPM, 0.34 Nm
 *      - NeveRest 20: 340 RPM, 0.17 Nm
 */