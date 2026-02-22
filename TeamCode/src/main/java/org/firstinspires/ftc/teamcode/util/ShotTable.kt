package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config

/**
 * Shot parameter lookup table: distance (metres) → (RPM, hood angle °).
 *
 * Tuning workflow
 * ---------------
 * 1. Place the robot at a known distance from the goal (measure with a tape).
 * 2. Run OuttakeTest in MANUAL mode and adjust RPM + hood angle until the ball
 *    lands in the goal consistently.
 * 3. Open FTC Dashboard → ShotTable, set the matching DIST_N, RPM_N, HOOD_N.
 * 4. Repeat at the next distance.  8 rows cover 1.0 m → 3.5 m in 0.35 m steps.
 * 5. Switch OuttakeTest to AUTO mode — the robot interpolates between your points.
 *
 * All constants are @JvmField so they appear in FTC Dashboard immediately.
 * Distances must be kept in ascending order for interpolation to work correctly.
 *
 * Outside the tuned range the table clamps to the nearest endpoint (no extrapolation).
 */
@Config
object ShotTable {

    // Row 0 — closest shot (~1.00 m)
    @JvmField var DIST_0 = 1.00; @JvmField var RPM_0 = 0.0; @JvmField var HOOD_0 = 20.0
    // Row 1
    @JvmField var DIST_1 = 1.35; @JvmField var RPM_1 = 0.0; @JvmField var HOOD_1 = 20.0
    // Row 2
    @JvmField var DIST_2 = 1.70; @JvmField var RPM_2 = 0.0; @JvmField var HOOD_2 = 20.0
    // Row 3
    @JvmField var DIST_3 = 2.05; @JvmField var RPM_3 = 0.0; @JvmField var HOOD_3 = 20.0
    // Row 4
    @JvmField var DIST_4 = 2.40; @JvmField var RPM_4 = 0.0; @JvmField var HOOD_4 = 20.0
    // Row 5
    @JvmField var DIST_5 = 2.75; @JvmField var RPM_5 = 0.0; @JvmField var HOOD_5 = 20.0
    // Row 6
    @JvmField var DIST_6 = 3.10; @JvmField var RPM_6 = 0.0; @JvmField var HOOD_6 = 20.0
    // Row 7 — furthest shot (~3.50 m)
    @JvmField var DIST_7 = 3.50; @JvmField var RPM_7 = 0.0; @JvmField var HOOD_7 = 20.0

    /**
     * Whether any rows have been tuned yet (all RPMs zero = untouched table).
     * Use this to show a warning in telemetry before attempting auto-aim.
     */
    val isTuned: Boolean
        get() = RPM_0 > 0.0 || RPM_1 > 0.0 || RPM_2 > 0.0 || RPM_3 > 0.0 ||
                RPM_4 > 0.0 || RPM_5 > 0.0 || RPM_6 > 0.0 || RPM_7 > 0.0

    /**
     * Look up (RPM, hoodAngle) for [distMeters].
     *
     * Clamps to the nearest endpoint outside the tuned range.
     * Linearly interpolates between the two nearest rows within the range.
     *
     * @return Pair(rpm, hoodAngleDegrees)
     */
    fun lookup(distMeters: Double): Pair<Double, Double> {
        // Build snapshot arrays so Dashboard tweaks mid-lookup don't cause inconsistency.
        val dists = doubleArrayOf(DIST_0, DIST_1, DIST_2, DIST_3, DIST_4, DIST_5, DIST_6, DIST_7)
        val rpms  = doubleArrayOf(RPM_0,  RPM_1,  RPM_2,  RPM_3,  RPM_4,  RPM_5,  RPM_6,  RPM_7)
        val hoods = doubleArrayOf(HOOD_0, HOOD_1, HOOD_2, HOOD_3, HOOD_4, HOOD_5, HOOD_6, HOOD_7)

        // Clamp below first entry.
        if (distMeters <= dists[0]) return Pair(rpms[0], hoods[0])
        // Clamp above last entry.
        if (distMeters >= dists[dists.size - 1]) return Pair(rpms[rpms.size - 1], hoods[hoods.size - 1])

        // Find the bracket [i, i+1] that contains distMeters.
        for (i in 0 until dists.size - 1) {
            if (distMeters <= dists[i + 1]) {
                val t = (distMeters - dists[i]) / (dists[i + 1] - dists[i])
                val rpm  = rpms[i]  + t * (rpms[i + 1]  - rpms[i])
                val hood = hoods[i] + t * (hoods[i + 1] - hoods[i])
                return Pair(rpm, hood)
            }
        }

        // Should never reach here, but be safe.
        return Pair(rpms[rpms.size - 1], hoods[hoods.size - 1])
    }
}
