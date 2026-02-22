package org.firstinspires.ftc.teamcode.enums

enum class AprilTag(val id: Int) {
    RED(24),
    BLUE(20),
    GPP(21),
    PGP(22),
    PPG(23);

    companion object {
        fun fromId(id: Int): AprilTag? {
            return entries.find { it.id == id }
        }
    }
}