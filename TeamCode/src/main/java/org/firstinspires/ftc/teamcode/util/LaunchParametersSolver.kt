package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import kotlin.math.*

object LaunchParametersSolver {
    private const val G = 9.81
    private const val K_DRAG = 0.0054
    private const val K_LIFT = 0.0046

    private const val GOAL_HEIGHT = 1.175
    private const val AXLE_HEIGHT = 1.1
    private const val RELATIVE_TARGET_Y = GOAL_HEIGHT - AXLE_HEIGHT
    private const val PIVOT_L = 0.1035

    private const val MIN_ANGLE = 20
    private const val MAX_ANGLE = 50
    private const val MAX_RPM = 5000.0
    private const val EFFICIENCY = 0.80
    private const val WHEEL_CIRCUMFERENCE = 0.3015928947

    private const val DT = 0.01
    private const val TOLERANCE = 0.02

    data class ShotResult(val angle: Int, val rpm: Double, val velocity: Double)

    /**
     * Entry point. Uses Ternary Search to find the angle with minimum RPM.
     */
    fun solve(targetDist: Double): ShotResult? {
        var low = MIN_ANGLE
        var high = MAX_ANGLE
        var globalBestRpm = Double.MAX_VALUE
        var globalBestSol: ShotResult? = null
        while (high - low > 2) {
            val m1 = low + (high - low) / 3
            val m2 = high - (high - low) / 3

            val res1 = evaluateAngle(m1, targetDist)
            val res2 = evaluateAngle(m2, targetDist)

            if (res1 != null && res1.rpm < globalBestRpm) {
                globalBestRpm = res1.rpm
                globalBestSol = res1
            }
            if (res2 != null && res2.rpm < globalBestRpm) {
                globalBestRpm = res2.rpm
                globalBestSol = res2
            }

            val rpm1 = res1?.rpm ?: Double.MAX_VALUE
            val rpm2 = res2?.rpm ?: Double.MAX_VALUE

            if (rpm1 < rpm2) {
                high = m2
            } else {
                low = m1
            }
        }

        for (a in low..high) {
            val res = evaluateAngle(a, targetDist)
            if (res != null && res.rpm < globalBestRpm) {
                globalBestRpm = res.rpm
                globalBestSol = res
            }
        }

        return globalBestSol
    }

    /**
     * Solves required Velocity/RPM for a specific angle using Secant Method.
     */
    private fun evaluateAngle(angle: Int, targetDist: Double): ShotResult? {
        val rads = Math.toRadians(angle.toDouble())
        val cosT = cos(rads)
        val sinT = sin(rads)

        var v0 = 10.0
        var v1 = 12.0 
        
        var y0 = runRK2(v0, cosT, sinT, targetDist)
        var y1 = runRK2(v1, cosT, sinT, targetDist)

        var vFound: Double? = null

        for (i in 0..7) {
            if (abs(y1) < TOLERANCE) {
                vFound = v1
                break
            }
            
            val slope = (y1 - y0) / (v1 - v0)
            if (abs(slope) < 1e-5) break
            
            val vNew = v1 - (y1 / slope)

            v0 = v1
            y0 = y1
            v1 = vNew

            if (v1 < 1.0) v1 = 1.0
            if (v1 > 40.0) v1 = 40.0
            
            y1 = runRK2(v1, cosT, sinT, targetDist)
        }

        if (vFound != null && abs(y1) < TOLERANCE) {
            val rpm = (2 * vFound / EFFICIENCY) * 60 / WHEEL_CIRCUMFERENCE
            if (rpm <= MAX_RPM) {
                return ShotResult(angle, rpm, vFound)
            }
        }
        return null
    }

    /**
     * RUNGE-KUTTA 2 (Heun's Method) INTEGRATOR
     * This is the "secret sauce" for speed.
     * It samples the slope at the start and end of the step and averages them.
     * Allows DT to be 0.01s instead of 0.001s.
     */
    private inline fun runRK2(vMag: Double, cosT: Double, sinT: Double, targetX: Double): Double {
        var x = PIVOT_L * cosT
        var y = PIVOT_L * sinT
        var vx = vMag * cosT
        var vy = vMag * sinT
        
        val floor = -AXLE_HEIGHT

        var step = 0
        while (x < targetX && step < 500) {

            val vSq = vx*vx + vy*vy
            val v = sqrt(vSq)

            val dragF = -K_DRAG * v
            val liftF = K_LIFT * v
            
            val ax1 = dragF * vx - liftF * vy
            val ay1 = dragF * vy + liftF * vx - G

            val vx_end = vx + ax1 * DT
            val vy_end = vy + ay1 * DT

            val vSq2 = vx_end*vx_end + vy_end*vy_end
            val v2 = sqrt(vSq2)
            
            val dragF2 = -K_DRAG * v2
            val liftF2 = K_LIFT * v2
            
            val ax2 = dragF2 * vx_end - liftF2 * vy_end
            val ay2 = dragF2 * vy_end + liftF2 * vx_end - G

            vx += (ax1 + ax2) * 0.5 * DT
            vy += (ay1 + ay2) * 0.5 * DT
            x += (vx + vx_end) * 0.5 * DT
            y += (vy + vy_end) * 0.5 * DT

            if (y < floor) return -999.0
            step++
        }
        
        return y - RELATIVE_TARGET_Y
    }
}
