package org.firstinspires.ftc.teamcode.utils.units

import android.annotation.SuppressLint
import org.firstinspires.ftc.teamcode.utils.units.Ang.Companion.chop
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.sign

data class Ang(private val _angle: Double) {
    companion object {
        fun chop(ang: Double): Double {
            var chopedAng = ang

            while (chopedAng.absoluteValue > PI)
                chopedAng -= 2 * PI * chopedAng.sign

            return chopedAng
        }

        val ZERO
            get() = Ang(0.0)

        fun ofDeg(angle: Double) = Ang(angle / 180.0 * PI)
    }

    val angle = chop(_angle)

    fun ofDegree() = angle / PI * 180

    override fun equals(other: Any?): Boolean {
        if (other == null)
            return false

        if (other is Ang && abs(other.angle - angle) < 0.005)
            return true

        return false
    }

    override fun hashCode(): Int {
        return angle.hashCode()
    }

    operator fun plus(ang: Ang) = Ang(chop(angle + ang.angle))
    operator fun plus(ang: Double) = Ang(chop(angle + ang))

    operator fun minus(ang: Ang) = Ang(chop(angle - ang.angle))
    operator fun minus(ang: Double) = Ang(chop(angle - ang))

    operator fun times(ang: Ang) = Ang(chop(angle * ang.angle))
    operator fun times(ang: Double) = Ang(chop(angle * ang))

    operator fun div(ang: Ang) = Ang(chop(angle / ang.angle))
    operator fun div(ang: Double) = Ang(chop(angle / ang))

    @SuppressLint("DefaultLocale")
    override fun toString(): String {
        return String.format("%.3f", ofDegree()) + "°"
    }

    fun clone() = Ang(angle)

    fun toDouble() = angle
}

operator fun Double.plus(ang: Ang) = Ang(chop(this + ang.angle))
operator fun Double.minus(ang: Ang) = Ang(chop(this - ang.angle))
operator fun Double.times(ang: Ang) = Ang(chop(this * ang.angle))
operator fun Double.div(ang: Ang) = Ang(chop(this / ang.angle))