package org.firstinspires.ftc.teamcode.utils.units

import java.util.Locale

data class Color(val r: Int, val g: Int, val b: Int) {
    companion object {
        val RED
            get() = Color(255, 0, 0)
        val BLUE
            get() = Color(0, 0, 255)
        val GREEN
            get() = Color(0, 255, 0)
        val GRAY
            get() = Color(128, 128, 128)
        val BLACK
            get() = Color(0, 0, 0)
        val WHITE
            get() = Color(255, 255, 255)
        val YELLOW
            get() = Color(255, 255, 0)
        val ORANGE
            get() = Color(255, 128, 0)
    }

    fun toHexString(): String {
        if (r > 255 || g > 255 || b > 255) throw RuntimeException("color more 255")

        if (r < 0 || g < 0 || b < 0) throw RuntimeException("color less 0")

        var rString = r.toString(16).uppercase(Locale.getDefault())
        var gString: String = g.toString(16).uppercase(Locale.getDefault())
        var bString: String = b.toString(16).uppercase(Locale.getDefault())

        if (rString.length == 1) rString = "0$rString"

        if (gString.length == 1) gString = "0$gString"

        if (bString.length == 1) bString = "0$bString"

        return "#$rString$gString$bString"
    }

    override fun toString(): String {
        return "($r, $g, $b)"
    }

    override fun equals(other: Any?): Boolean {
        if (other == null)
            return false

        if (other !is Color)
            return false

        if (other.r == r && other.g == g && other.b == b)
            return true

        return false
    }

    fun clone() = Color(r, g, b)

    constructor() : this(0, 0, 0)

    override fun hashCode(): Int {
        var result = r
        result = 31 * result + g
        result = 31 * result + b
        return result
    }
}