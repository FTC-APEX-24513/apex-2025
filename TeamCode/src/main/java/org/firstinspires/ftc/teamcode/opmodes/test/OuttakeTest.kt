package org.firstinspires.ftc.teamcode.opmodes.test

import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.ifHuh
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.parallel
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.commands.OuttakeCommands
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.util.LaunchParametersSolver
import org.firstinspires.ftc.teamcode.util.ShotTable

/**
 * Test OpMode for the Outtake Subsystem.
 *
 * Controls
 * --------
 *   Triangle            — Toggle AUTO / MANUAL mode.
 *   Right Bumper        — Stop outtake (any mode).
 *
 *   Manual mode:
 *     Left  Stick Y     — Coarse flywheel RPM  (push forward = max RPM)
 *     Right Stick Y     — Coarse hood angle    (push forward = max angle)
 *     D-Pad Left/Right  — Fine RPM ±50 per press
 *     D-Pad Up/Down     — Fine hood angle ±1° per press
 *
 * Telemetry
 * ---------
 *   Mode, motor RPM, at-target flag.
 *   Distance from three sources (MT2, ty, pinpoint) side-by-side for calibration.
 *   Table lookup vs physics solver — both always shown for comparison.
 *   ShotTable tuning reminder when table has no data.
 */
@Suppress("UNUSED")
val outtakeTest = Mercurial.teleop {
    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    val outtake   = container.outtake
    val limelight = container.limelight

    var autoShoot  = false
    var hoodAngle  = OuttakeSubsystem.DEFAULT_HOOD_ANGLE
    var manualRPM  = 0.0

    val RPM_SCALE      = OuttakeSubsystem.MAX_RPM
    val FINE_RPM_STEP  = 50.0
    val FINE_HOOD_STEP = 1.0

    waitForStart()

    // Toggle AUTO / MANUAL.
    bindSpawn(risingEdge { gamepad1.triangle }, exec { autoShoot = !autoShoot })

    // Stop.
    bindSpawn(risingEdge { gamepad1.right_bumper }, parallel(
        outtake.stop(),
        exec { manualRPM = 0.0; hoodAngle = OuttakeSubsystem.DEFAULT_HOOD_ANGLE }
    ))

    // Fine adjustments (manual only).
    bindSpawn(risingEdge { gamepad1.dpad_right }, exec {
        if (!autoShoot) manualRPM = (manualRPM + FINE_RPM_STEP).coerceIn(0.0, OuttakeSubsystem.MAX_RPM)
    })
    bindSpawn(risingEdge { gamepad1.dpad_left }, exec {
        if (!autoShoot) manualRPM = (manualRPM - FINE_RPM_STEP).coerceIn(0.0, OuttakeSubsystem.MAX_RPM)
    })
    bindSpawn(risingEdge { gamepad1.dpad_up }, exec {
        if (!autoShoot) hoodAngle = (hoodAngle + FINE_HOOD_STEP)
            .coerceIn(OuttakeSubsystem.MIN_HOOD_ANGLE, OuttakeSubsystem.MAX_HOOD_ANGLE)
    })
    bindSpawn(risingEdge { gamepad1.dpad_down }, exec {
        if (!autoShoot) hoodAngle = (hoodAngle - FINE_HOOD_STEP)
            .coerceIn(OuttakeSubsystem.MIN_HOOD_ANGLE, OuttakeSubsystem.MAX_HOOD_ANGLE)
    })

    // -------------------------------------------------------------------------
    // Main control loop.
    //
    // AUTO:   aimOuttake() is called each tick — it recalculates distance and
    //         looks up the table, then returns a spinUp or stop Closure.
    //
    // MANUAL: stick/dpad values are read into manualRPM/hoodAngle each tick,
    //         then the lazy spinUp overload sends those current values to the
    //         subsystem (evaluated at send time, not at loop-build time).
    // -------------------------------------------------------------------------
    schedule(loop({ inLoop },
        ifHuh({ autoShoot },
            OuttakeCommands.aimOuttake(limelight, outtake)
        ).elseHuh(
            parallel(
                // Update manualRPM / hoodAngle from sticks.
                exec {
                    val stickRPM  = gamepad1.left_stick_y.toDouble()
                    val stickHood = -gamepad1.right_stick_y.toDouble()

                    val coarseRPM  = (-stickRPM).coerceIn(0.0, 1.0) * RPM_SCALE
                    val hoodRange  = OuttakeSubsystem.MAX_HOOD_ANGLE - OuttakeSubsystem.MIN_HOOD_ANGLE
                    val coarseHood = OuttakeSubsystem.MIN_HOOD_ANGLE + ((stickHood + 1.0) / 2.0) * hoodRange

                    if (stickRPM  < -0.1 || stickRPM  > 0.1) manualRPM = coarseRPM
                    if (stickHood < -0.1 || stickHood > 0.1) hoodAngle  = coarseHood
                },
                // Send the current values to the subsystem every tick.
                // Lazy lambdas read manualRPM / hoodAngle at the moment the channel sends.
                outtake.spinUp({ manualRPM }, { hoodAngle })
            )
        )
    ))

    // -------------------------------------------------------------------------
    // Telemetry loop.
    // -------------------------------------------------------------------------
    schedule(loop({ inLoop }, exec {

        // Gather all distance estimates.
        val distMT2      = limelight.getDistanceMT2()
        val distTy       = limelight.getDistanceTy(OuttakeCommands.GOAL_TAG_ID)
        val distPinpoint = limelight.getDistancePinpoint()
        val bestDist     = limelight.getDistanceToGoal(OuttakeCommands.GOAL_TAG_ID)

        // Table lookup at the best distance.
        val tableResult = if (ShotTable.isTuned)
            ShotTable.lookup(bestDist.distanceMeters) else null

        // Physics solver at the best distance.
        val physicsResult = LaunchParametersSolver
            .solve(bestDist.distanceMeters)

        telemetry.addLine("=== OUTTAKE TEST ===")
        telemetry.addData("Mode", if (autoShoot) "AUTO  [triangle to toggle]"
                                  else            "MANUAL [triangle to toggle]")
        telemetry.addLine("")

        // --- Distance sources ---
        telemetry.addLine("--- Distance to Goal ---")
        telemetry.addData("MT2      (needs tags)  ", distMT2?.let { "%.3f m".format(it) } ?: "unavailable")
        telemetry.addData("ty       (needs tag)   ", distTy?.let  { "%.3f m".format(it) } ?: "unavailable")
        telemetry.addData("Pinpoint (always avail)", "%.3f m".format(distPinpoint))
        telemetry.addData("Active   (${bestDist.source.name})", "%.3f m".format(bestDist.distanceMeters))
        telemetry.addLine("")

        // --- Shot parameters (both solvers, always visible for comparison) ---
        telemetry.addLine("--- Shot Parameters ---")
        telemetry.addData("Active solver", if (OuttakeCommands.USE_PHYSICS_SOLVER) "PHYSICS" else "TABLE")
        if (tableResult != null) {
            telemetry.addData("Table  RPM ", "%.0f".format(tableResult.first))
            telemetry.addData("Table  hood", "%.1f deg".format(tableResult.second))
        } else {
            telemetry.addData("Table", "NOT TUNED — set RPM_N values in Dashboard > ShotTable")
        }
        if (physicsResult != null) {
            telemetry.addData("Physics RPM ", "%.0f".format(physicsResult.rpm))
            telemetry.addData("Physics hood", "%d deg".format(physicsResult.angle))
            telemetry.addData("Physics vel ", "%.2f m/s".format(physicsResult.velocity))
        } else {
            telemetry.addData("Physics", "no solution at this distance")
        }
        telemetry.addLine("")

        // --- Motor feedback ---
        telemetry.addLine("--- Motor Feedback ---")
        telemetry.addData("Current RPM (avg)", "%.0f".format(outtake.getCurrentRPM()))
        telemetry.addData("Motor 0 RPM",       "%.0f".format(outtake.getMotor0RPM()))
        telemetry.addData("Motor 1 RPM",       "%.0f".format(outtake.getMotor1RPM()))
        telemetry.addData("At target RPM?",    outtake.isAtTargetRPM())
        telemetry.addLine("")

        // --- Manual set-points ---
        if (!autoShoot) {
            telemetry.addLine("--- Manual Set-points ---")
            telemetry.addData("Set RPM",    "%.0f".format(manualRPM))
            telemetry.addData("Hood angle", "%.1f deg".format(hoodAngle))
            telemetry.addLine("  L-stick Y = coarse RPM  | D-Left/Right = ±50 RPM")
            telemetry.addLine("  R-stick Y = coarse hood | D-Up/Down    = ±1 deg")
            telemetry.addLine("  RB = stop")
        }

        telemetry.update()
    }))

    dropToScheduler()
}
