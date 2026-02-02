package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.mercurial.continuations.Closure
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareScope
import org.firstinspires.ftc.teamcode.util.VoltageCompensation
import kotlin.math.abs
import kotlin.math.round

@Configurable
@Inject
@HardwareScope
class SpindexerSubsystem(hardwareMap: HardwareMap, private val voltageCompensation: VoltageCompensation) : Subsystem() {
    private val spindexer = hardwareMap.crservo.get("spindexer")
    private val encoder = hardwareMap.get(DcMotorEx::class.java, "spindexerREVEncoder").apply {
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    companion object {
        @JvmField var TICKS_PER_REV = 8192.0
//        @JvmField var KP = 0.00029659
//        @JvmField var KI = 0.00058751
//        @JvmField var KD = 0.00003743
        @JvmField var KP = 0.00064400
        @JvmField var KI = 0.00015908
        @JvmField var KD = 0.00008147
        @JvmField var KF = 0.0
        @JvmField var ALLOWED_ERROR = 50.0
        @JvmField var IDLE_POWER = 0.0
    }

    sealed interface State {
        object Idle : State
        data class Targeting(val targetPosition: Double) : State
        data class Manual(val power: Double) : State
    }

    var state: State = State.Idle
        private set

    private var previousError = 0.0
    private var integralSum = 0.0
    private var lastTime = System.nanoTime()

    private val ticksPerInterval: Double
        get() = TICKS_PER_REV / 6.0

    override fun periodic(): Closure = exec {
        val currentPosition = encoder.currentPosition.toDouble()
        val currentTime = System.nanoTime()
        val dt = (currentTime - lastTime) / 1e9
        lastTime = currentTime

        val power = when (val s = state) {
            is State.Idle -> {
                integralSum = 0.0
                previousError = 0.0
                IDLE_POWER
            }
            is State.Manual -> {
                integralSum = 0.0
                previousError = 0.0
                s.power
            }
            is State.Targeting -> {
                val error = s.targetPosition - currentPosition
                
                if (abs(error) < ALLOWED_ERROR) {
                    integralSum = 0.0
                    previousError = 0.0
                    0.0
                } else {
                    integralSum += error * dt
                    val derivative = (error - previousError) / dt
                    previousError = error

                    val pidOutput = (error * KP) + (integralSum * KI) + (derivative * KD) + (s.targetPosition * KF) // KF often scales with target velocity or position depending on system
                    // For position hold, KF usually compensates gravity (constant) or is 0. 
                    // If targetPosition implies motion, KF might be useful. 
                    // Here we just add it as requested.
                    
                    pidOutput.coerceIn(-1.0, 1.0)
                }
            }
        }
        
        spindexer.power = power
    }

    fun advance(): Closure = exec {
        val currentPosition = encoder.currentPosition.toDouble()
        // Calculate next slot: round current to nearest slot, then add 1 interval
        // Or strictly: floor(current / interval) + 1 ?
        // If we are sitting exactly at a slot, we want the next one.
        // If we are slightly past a slot, we want the next one.
        val currentSlot = round(currentPosition / ticksPerInterval)
        val targetSlot = currentSlot + 1
        state = State.Targeting(targetSlot * ticksPerInterval)
    }
    
    fun reverse(): Closure = exec {
        val currentPosition = encoder.currentPosition.toDouble()
        val currentSlot = round(currentPosition / ticksPerInterval)
        val targetSlot = currentSlot - 1
        state = State.Targeting(targetSlot * ticksPerInterval)
    }

    fun stop(): Closure = exec { state = State.Idle }
    
    fun setPower(power: Double): Closure = exec { state = State.Manual(power) }
    
    // Helper to get current slot index
    fun getCurrentSlot(): Int {
        val currentPosition = encoder.currentPosition.toDouble()
        // Normalize to 0-5
        val slot = round(currentPosition / ticksPerInterval).toLong() % 6
        return if (slot < 0) (slot + 6).toInt() else slot.toInt()
    }
    
    // Direct power control for auto-tuning (bypasses periodic)
    fun setDirectPower(power: Double) {
        spindexer.power = power.coerceIn(-1.0, 1.0)
    }
}
