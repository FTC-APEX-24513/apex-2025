package org.firstinspires.ftc.teamcode.subsystems

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.match
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister
import me.tatarka.inject.annotations.Inject
import org.firstinspires.ftc.teamcode.di.HardwareFactory
import org.firstinspires.ftc.teamcode.di.HardwareScope
import org.firstinspires.ftc.teamcode.enums.BallColor

@Config
@Inject
@HardwareScope
class IndexerSubsystem(factory: HardwareFactory) : Subsystem<IndexerSubsystem.State, IndexerSubsystem.State>() {

    companion object {
        @JvmField
        var MIN_ALPHA_DETECT = 0.5

        @JvmField
        var SENSOR_GAIN = 25.0f

        @JvmField
        var GREEN_HUE_MIN = 140.0

        @JvmField
        var GREEN_HUE_MAX = 180.0

        @JvmField
        var PURPLE_HUE_MIN = 217.0

        @JvmField
        var PURPLE_HUE_MAX = 260.0
    }

    private val slots = arrayOf(
        IndexerSlot(factory.getColorSensor("colorSensor00"), factory.getColorSensor("colorSensor01")),
        IndexerSlot(factory.getColorSensor("colorSensor10"), factory.getColorSensor("colorSensor11")),
//        IndexerSlot(factory.getColorSensor("colorSensor20"), factory.getColorSensor("colorSensor21"))
    )

    val inventory = Array(3) { BallColor.NONE }

    sealed interface State {
        object Scanning : State
        object Idle : State
    }

    override val initialState = { State.Scanning }
    override val transition = { _: State, msg: State -> msg }

    override val behavior = { register: VarRegister<State> ->
        match { register.get() }.branch(State.Scanning, loop(exec {
            for (i in slots.indices) {
                inventory[i] = slots[i].detectColor()
            }
        })).branch(State.Idle, loop(noop())).assertExhaustive()
    }

    fun getOrder(order: Array<BallColor>): List<Int> {
        val tempInventory = inventory.clone()
        val targets = ArrayList<Int>(order.size)

        for (requestedColor in order) {
            var foundIndex = -1
            for (i in tempInventory.indices) {
                if (tempInventory[i] == requestedColor) {
                    foundIndex = i
                    break
                }
            }
            if (foundIndex != -1) {
                targets.add(foundIndex)
                tempInventory[foundIndex] = BallColor.NONE
            } else {
                targets.add(-1)
            }
        }
        return targets
    }

    private class IndexerSlot(val s1: NormalizedColorSensor, val s2: NormalizedColorSensor) {
        private val hsv = FloatArray(3)

        init {
            s1.gain = SENSOR_GAIN
            s2.gain = SENSOR_GAIN
        }

        fun detectColor(): BallColor {
            val c1 = s1.normalizedColors
            val c2 = s2.normalizedColors

            val r1 = resolveReading(c1)
            val r2 = resolveReading(c2)


            if (r1 != BallColor.NONE && r2 == BallColor.NONE) return r1

            if (r2 != BallColor.NONE && r1 == BallColor.NONE) return r2

            if (r1 != BallColor.NONE) {
                if (r1 != r2) {
                    return if (c1.alpha > c2.alpha) r1 else r2
                }
                return r1
            }

            return BallColor.NONE
        }

        /**
         * Converts a raw reading into a Color or NONE.
         */
        private fun resolveReading(c: NormalizedRGBA): BallColor {
            if (c.alpha < MIN_ALPHA_DETECT) {
                return BallColor.NONE
            }

            Color.RGBToHSV(
                (c.red * 255).toInt(),
                (c.green * 255).toInt(),
                (c.blue * 255).toInt(),
                hsv
            )
            val hue = hsv[0]

            return when (hue) {
                in GREEN_HUE_MIN..GREEN_HUE_MAX -> BallColor.GREEN
                in PURPLE_HUE_MIN..PURPLE_HUE_MAX -> BallColor.PURPLE
                else -> BallColor.NONE
            }
        }
    }
}