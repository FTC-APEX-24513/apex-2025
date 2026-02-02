package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

@Suppress("UNUSED")
val spindexerAutoTuner = Mercurial.teleop {
    // Get direct servo access to bypass subsystem during tuning
    val servo = hardwareMap.crservo.get("spindexer")
    val encoder = hardwareMap.get(DcMotorEx::class.java, "spindexerREVEncoder").apply {
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    // Tuning Parameters
    val tuneStepPower = 0.4 // Power applied during relay step
    val tuningCycles = 5    // Number of oscillation cycles to average
    val hysteresis = 100.0  // Reduced hysteresis - easier to trigger
    val maxSwitchTime = 5.0 // Maximum seconds per half-cycle before forcing switch

    // Fine-tuning multipliers (adjust after auto-tune)
    var kpMultiplier = 1.0 // Increase for snappier response
    var kdMultiplier = 1.5 // Increase derivative for better damping
    var kiMultiplier = 0.3 // Reduce integral to prevent windup
    
    // State Variables
    var tuningInProgress = false
    var tuningComplete = false
    var halfCycleCount = 0  // Count half-cycles (direction changes)
    var peaks = mutableListOf<Double>()
    var valleys = mutableListOf<Double>()
    var periods = mutableListOf<Double>()
    
    var lastSwitchTime = 0.0
    var startPosition = 0.0
    var direction = 1 // 1 = moving positive, -1 = moving negative
    var lastPeakValleyPosition = 0.0 // Store position at last peak or valley
    
    var calculatedKp = 0.0
    var calculatedKi = 0.0
    var calculatedKd = 0.0
    var calculatedKf = 0.0

    // Position history for velocity calculation
    var positionHistory = mutableListOf<Pair<Double, Double>>() // (time, position)
    
    // Current power to apply (updated during tuning)
    var currentPower = 0.0
    
    // Debug tracking
    var timeSinceSwitch = 0.0
    var lastVelocity = 0.0

    val container = HardwareContainer::class.create(hardwareMap, scheduler).also {
        it.startPeriodic()
    }

    waitForStart()

    // Control Bindings
    // Press A to start Auto-Tune
    bindSpawn(risingEdge { gamepad1.a && !tuningInProgress }, exec {
        tuningInProgress = true
        tuningComplete = false
        halfCycleCount = 0
        peaks.clear()
        valleys.clear()
        periods.clear()
        positionHistory.clear()
        
        startPosition = encoder.currentPosition.toDouble()
        lastSwitchTime = System.nanoTime() / 1e9
        direction = 1
        lastPeakValleyPosition = startPosition
        positionHistory.add(Pair(lastSwitchTime, startPosition))
        timeSinceSwitch = 0.0
        lastVelocity = 0.0
        
        // Set power to be applied continuously in the main loop
        currentPower = tuneStepPower * direction
        
        telemetry.addLine("Auto-tuning started!")
        telemetry.update()
    })

    // Press B to Stop/Reset
    bindSpawn(risingEdge { gamepad1.b }, exec {
        if (tuningInProgress) {
            tuningInProgress = false
            currentPower = 0.0
            container.spindexer.stop()
        }
    })
    
//    // Fine-tuning controls (after tuning) - simplified without helper function
//    bindSpawn(loop({ tuningComplete && !tuningInProgress }, exec {
//        // D-pad to adjust Kp multiplier
//        if (gamepad1.dpad_up && gamepad1.a) {
//            kpMultiplier += 0.1
//            applyMultipliers()
//        }
//        if (gamepad1.dpad_down && gamepad1.a) {
//            kpMultiplier = max(0.1, kpMultiplier - 0.1)
//            applyMultipliers()
//        }
//
//        // D-pad to adjust Kd multiplier
//        if (gamepad1.dpad_right && gamepad1.a) {
//            kdMultiplier += 0.1
//            applyMultipliers()
//        }
//        if (gamepad1.dpad_left && gamepad1.a) {
//            kdMultiplier = max(0.1, kdMultiplier - 0.1)
//            applyMultipliers()
//        }
//
//        // D-pad to adjust Ki multiplier
//        if (gamepad1.y) {
//            kiMultiplier += 0.1
//            applyMultipliers()
//        }
//        if (gamepad1.x) {
//            kiMultiplier = max(0.0, kiMultiplier - 0.1)
//            applyMultipliers()
//        }
//    }))
    
    // Manual testing bindings (available after tuning or if stopped)
    bindSpawn(risingEdge { gamepad1.right_bumper }, container.spindexer.advance())
    bindSpawn(risingEdge { gamepad1.left_bumper }, container.spindexer.reverse())

    schedule(
        loop({ inLoop }, exec {
            val currentPos = encoder.currentPosition.toDouble()
            val time = System.nanoTime() / 1e9
            timeSinceSwitch = time - lastSwitchTime

            // CONTINUOUSLY OVERRIDE SERVO POWER DURING TUNING
            // This must happen before subsystem periodic() overrides it
            if (tuningInProgress) {
                servo.power = currentPower
            }

            if (tuningInProgress) {
                // --- RELAY TUNING LOGIC ---
                positionHistory.add(Pair(time, currentPos))
                if (positionHistory.size > 15) positionHistory.removeAt(0)
                
                val error = startPosition - currentPos
                val errorDirection = if (direction == 1) -hysteresis else hysteresis
                
                // Calculate instantaneous velocity
                val velocity = if (positionHistory.size >= 3) {
                    val recent = positionHistory.takeLast(3)
                    (recent.last().second - recent.first().second) / (recent.last().first - recent.first().first)
                } else {
                    0.0
                }
                lastVelocity = velocity
                
                // Track extreme positions during current half-cycle
                if (direction == 1 && currentPos > lastPeakValleyPosition) {
                    lastPeakValleyPosition = currentPos
                } else if (direction == -1 && currentPos < lastPeakValleyPosition) {
                    lastPeakValleyPosition = currentPos
                }
                
                // DIRECTION CHANGE DETECTION - Multiple triggers for reliability:
                // 1. Velocity has crossed zero (turnaround detected)
                // 2. OR error crossed hysteresis threshold
                // 3. OR time limit exceeded (safety fallback)
                
                val velocityCrossedZero = if (direction == 1) velocity < -50 else velocity > 50
                val errorCrossedThreshold = if (direction == 1) error < -hysteresis else error > hysteresis
                val timeExceeded = timeSinceSwitch > maxSwitchTime
                
                val shouldSwitch = velocityCrossedZero || errorCrossedThreshold || timeExceeded
                
                if (shouldSwitch && timeSinceSwitch > 0.5) { // Minimum 0.5s before switching
                    // Direction change detected
                    val halfPeriodTime = timeSinceSwitch
                    
                    if (halfCycleCount > 0) {
                        // Record peak or valley
                        if (direction == 1) {
                            peaks.add(lastPeakValleyPosition)
                        } else {
                            valleys.add(lastPeakValleyPosition)
                        }
                        
                        // Every 2 direction changes = 1 full cycle
                        if (halfCycleCount % 2 == 0) {
                            periods.add(halfPeriodTime * 2)
                        }
                    } else {
                        // First time, reset peak/valley for accurate tracking
                        lastPeakValleyPosition = currentPos
                    }
                    
                    // Switch direction
                    direction *= -1
                    lastSwitchTime = time
                    lastPeakValleyPosition = currentPos
                    halfCycleCount++
                    
                    // Update power for next loop iteration
                    currentPower = tuneStepPower * direction
                }

                // Completion Check
                if (halfCycleCount >= tuningCycles * 2 && periods.size >= tuningCycles) {
                    tuningInProgress = false
                    tuningComplete = true
                    currentPower = 0.0
                    
                    // --- ZIEGLER-NICHOLS CALCULATION WITH SERVO OPTIMIZATION ---
                    // Ultimate Gain: Ku = 4d / (pi * a)
                    // d = step amplitude (power)
                    // a = oscillation amplitude (ticks)
                    
                    if (peaks.isNotEmpty() && valleys.isNotEmpty()) {
                        val avgPeak = peaks.average()
                        val avgValley = valleys.average()
                        val amplitude = (avgPeak - avgValley) / 2.0
                        val avgPeriod = periods.average()
                        
                        if (amplitude > 0 && avgPeriod > 0) {
                            val Ku = (4.0 * tuneStepPower) / (PI * amplitude)
                            
                            // SERVO-ADJUSTED Ziegler-Nichols:
                            // Standard Z-N is conservative for velocity control
                            // Servos need higher Kp for responsiveness, lower Ki to prevent windup
                            
                            calculatedKp = 1.2 * Ku // Increased from 0.6 for snappier response
                            calculatedKi = 0.3 * Ku / avgPeriod // Reduced from 1.2 to prevent overshoot
                            calculatedKd = 0.15 * Ku * avgPeriod // Increased from 0.075 for better damping
                            
                            // Apply with fine-tuning multipliers
//                            applyMultipliers()
                            
                            telemetry.addLine("=== TUNING COMPLETE ===")
                            telemetry.addLine("Press A + D-pad to fine-tune:")
                            telemetry.addLine("  D-pad UP/DOWN: Adjust Kp")
                            telemetry.addLine("  D-pad RIGHT/LEFT: Adjust Kd")
                            telemetry.addLine("  Y/X: Adjust Ki")
                            telemetry.addLine("  LB/RB: Test advance/reverse")
                            telemetry.update()
                        }
                    }
                }
            }

            // --- TELEMETRY ---
            telemetry.addLine("=== Spindexer Auto-Tuner (Axon Mini+) ===")
            if (tuningInProgress) {
                telemetry.addData("Status", "TUNING... Cycle ${halfCycleCount/2}/$tuningCycles")
                telemetry.addData("Time since switch", "%.2fs", timeSinceSwitch)
                telemetry.addData("Displacement", "%.1f", currentPos - startPosition)
                telemetry.addData("Velocity", "%.1f", lastVelocity)
                telemetry.addLine("Switch triggers:")
                telemetry.addData("  Velocity zero", if (direction == 1) lastVelocity < -50 else lastVelocity > 50)
                telemetry.addData("  Error threshold", if (direction == 1) (startPosition - currentPos) < -hysteresis else (startPosition - currentPos) > hysteresis)
                telemetry.addData("  Time limit", timeSinceSwitch > maxSwitchTime)
            } else {
                telemetry.addData("Status", if (tuningComplete) "COMPLETE - Use A+D-pad to Fine-tune" else "IDLE - Press A to Start")
            }
            
            if (tuningComplete) {
                telemetry.addLine("\n=== Fine-Tuning (Hold A) ===")
                telemetry.addData("Kp Multiplier", "%.1fx", kpMultiplier)
                telemetry.addData("Kd Multiplier", "%.1fx", kdMultiplier)
                telemetry.addData("Ki Multiplier", "%.1fx", kiMultiplier)
            }
            
            telemetry.addLine("\n=== Calculated Values ===")
            telemetry.addData("Kp", "%.8f", calculatedKp)
            telemetry.addData("Ki", "%.8f", calculatedKi)
            telemetry.addData("Kd", "%.8f", calculatedKd)
            
            telemetry.addLine("\n=== Active in Subsystem ===")
            telemetry.addData("Kp", "%.8f", SpindexerSubsystem.KP)
            telemetry.addData("Ki", "%.8f", SpindexerSubsystem.KI)
            telemetry.addData("Kd", "%.8f", SpindexerSubsystem.KD)
            
            if (tuningInProgress) {
                telemetry.addLine("\n=== Debug ===")
                telemetry.addData("Position", "%.1f", currentPos)
                telemetry.addData("Start", "%.1f", startPosition)
                telemetry.addData("Peak/Valley", "%.1f", lastPeakValleyPosition)
                telemetry.addData("Peaks/Valleys", "${peaks.size}/${valleys.size}")
                telemetry.addData("Periods", periods.size)
            }
            
            telemetry.update()
        })
    )

    dropToScheduler()
}

// Helper function to reapply tuning with multipliers
fun reapplyTuning(kpMult: Double, kiMult: Double, kdMult: Double) {
    // This function is called from within the tuner scope, needs access to calculated values
    // For now, we'll inline this logic in the main tuner
}
