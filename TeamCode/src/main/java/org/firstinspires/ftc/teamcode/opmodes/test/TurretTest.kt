package org.firstinspires.ftc.teamcode.opmodes.test

import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.ifHuh
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.parallel
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.scope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.commands.TurretCommands
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem
import kotlin.math.abs

/**
 * Test OpMode for the Turret Subsystem.
 *
 * Use this OpMode to:
 *   1. Tune DEGREES_PER_SECOND — drive at full power for a known time, compare
 *      estimatedPosition to the actual rotation measured physically.
 *   2. Tune MIN_POWER / FULL_POWER_TX — enable tracking and adjust until the turret
 *      converges smoothly without oscillating.
 *
 * Controls
 * --------
 *   OPTIONS / Cross (any time) — EMERGENCY STOP: cuts power immediately.
 *
 *   Left Stick X           — Manual drive at proportional power (deadzone ±10%).
 *                            Only active when tracking is OFF.
 *
 *   Right Bumper           — Toggle tracking mode on/off.
 *                            ON  → continuous Limelight lockOnGoal.
 *                            OFF → manual L-stick drive.
 *
 *   Triangle               — Re-home: reset estimatedPosition to 0.0 at the current
 *                            physical position.
 *
 * Telemetry
 * ---------
 *   Emergency stop status, manual/tracking mode, Limelight tx, estimated position,
 *   tracking lock status.
 */
@Suppress("UNUSED")
val turretTest = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val turret    = container.turret
    val limelight = container.limelight

    var lastResult: TurretCommands.TrackingResult? = null
    var tracking = false

    val alliance = TurretCommands.Alliance.BLUE

    waitForStart()

    // -------------------------------------------------------------------------
    // Emergency stop — rising edge on OPTIONS or Cross cuts power once.
    // -------------------------------------------------------------------------
    bindSpawn(risingEdge { gamepad1.options || gamepad1.cross }, turret.hold())

    // -------------------------------------------------------------------------
    // Right Bumper — toggle tracking mode.
    // -------------------------------------------------------------------------
    bindSpawn(risingEdge { gamepad1.right_bumper }, exec {
        tracking = !tracking
        if (!tracking) lastResult = null
    })

    // -------------------------------------------------------------------------
    // Triangle — re-home.
    // -------------------------------------------------------------------------
    bindSpawn(risingEdge { gamepad1.triangle }, exec {
        turret.resetEstimate()
    })

    // -------------------------------------------------------------------------
    // Main control loop — tracking or manual drive, guarded by emergency stop.
    //
    // The manual drive branch uses scope { var stick by variable { 0.0 } } so that
    // the stick value is written into a register each tick by the exec, and then
    // read from that same register by turret.drive(stick) in the same sequence step.
    // -------------------------------------------------------------------------
    schedule(loop({ inLoop && !gamepad1.options && !gamepad1.cross },
        ifHuh(
            { tracking },
            // Tracking branch: lockOnGoal reads tx fresh each outer-loop tick and
            // sends a new Track/Search message so the turret continuously follows.
            TurretCommands.lockOnGoal(limelight, turret, alliance) { result ->
                lastResult = result
            }
        ).elseHuh(
            // Manual branch: scope keeps the stick value in a register so that
            // turret.drive(stick) reads the value written by exec this same tick.
            parallel(
                scope {
                    var stick by variable { 0.0 }
                    sequence(
                        exec { stick = gamepad1.left_stick_x.toDouble() },
                        ifHuh(
                            { abs(stick) > 0.1 },
                            turret.drive { stick }
                        ).elseHuh(
                            noop()
                        )
                    )
                },
                exec { lastResult = null }
            )
        )
    ))

    // -------------------------------------------------------------------------
    // Telemetry loop — runs independently every tick.
    // -------------------------------------------------------------------------
    schedule(loop({ inLoop }, exec {
        telemetry.addLine("=== TURRET TEST ===")

        if (gamepad1.options || gamepad1.cross) {
            telemetry.addLine("!! EMERGENCY STOP !!")
            telemetry.addLine("")
        }

        val mode = when {
            gamepad1.options || gamepad1.cross -> "EMERGENCY STOP"
            tracking                           -> "TRACKING  (RB to disable)"
            else                               -> "MANUAL    (RB to enable tracking)"
        }
        telemetry.addData("Mode", mode)
        telemetry.addLine("")

        telemetry.addLine("--- Position (dead-reckoning) ---")
        telemetry.addData("Estimated pos", "%.1f deg", turret.estimatedPosition)
        telemetry.addData("At limit?",     if (turret.isAtLimit) "YES" else "no")
        telemetry.addData("LIMIT_MIN",     "%.1f deg", TurretSubsystem.LIMIT_MIN)
        telemetry.addData("LIMIT_MAX",     "%.1f deg", TurretSubsystem.LIMIT_MAX)
        telemetry.addLine("")

        telemetry.addLine("--- Limelight / Tracking ---")
        lastResult?.let { r ->
            telemetry.addData("Source",      r.source)
            telemetry.addData("Tag visible", r.tagVisible)
            telemetry.addData("tx",          r.tx?.let { "%.2f°".format(it) } ?: "N/A")
            telemetry.addData("Locked",      r.isLocked)
            telemetry.addData("Staleness",   "${r.stalenessMs} ms")
        } ?: run {
            val tx = limelight.getTargetTxForTag(alliance.tagId)
            telemetry.addData("Tag visible", tx != null)
            telemetry.addData("tx",          tx?.let { "%.2f°".format(it) } ?: "N/A")
        }
        telemetry.addLine("")

        telemetry.addLine("--- Controls ---")
        telemetry.addLine("OPTIONS/Cross = emergency stop")
        telemetry.addLine("L-stick X     = manual drive (tracking OFF)")
        telemetry.addLine("RB            = toggle tracking on/off")
        telemetry.addLine("Triangle      = re-home (reset estimate to 0)")

        telemetry.update()
    }))

    dropToScheduler()
}
