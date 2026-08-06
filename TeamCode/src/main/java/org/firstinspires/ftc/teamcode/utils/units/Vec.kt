package org.firstinspires.ftc.teamcode.utils.units

import android.annotation.SuppressLint
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

data class Vec(val x: Double, val y: Double) {
    companion object {
        val ZERO
            get() = Vec(0.0, 0.0)
    }

    constructor(x: Double) : this(x, x)
    constructor(): this(0.0, 0.0)

    fun l() = hypot(x, y)

    fun rot() = atan2(y, x)

    fun setRot(rot: Double): Vec {
        val l = l()

        return Vec(cos(rot) * l, sin(rot) * l)
    }

    fun turn(rot: Double): Vec {
        val cosRot = cos(rot)
        val sinRot = sin(rot)

        return Vec(
            x * cosRot - y *sinRot,
            x * sinRot + y * cosRot
        )
    }

    fun normalized() = Vec(1.0, 0.0).setRot(rot())

    override fun equals(other: Any?): Boolean {
        if (other == null)
            return false

        if (other !is Vec)
            return false

        if (abs(other.x - x) < 0.005 && abs(other.y - y) < 0.005)
            return true

        return false
    }

    operator fun plus(vec: Vec) = Vec(x + vec.x, y + vec.y)
    operator fun minus(vec: Vec) = Vec(x - vec.x, y - vec.y)
    operator fun times(vec: Vec) = Vec(x * vec.x, y * vec.y)
    operator fun div(vec: Vec) = Vec(x / vec.x, y / vec.y)

    operator fun times(value: Double) = Vec(x * value, y * value)
    operator fun div(value: Double) = Vec(x / value, y / value)
    operator fun plus(value: Double) = Vec(x + value, y + value)
    operator fun minus(value: Double) = Vec(x - value, y - value)

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        return result
    }

    @SuppressLint("DefaultLocale")
    override fun toString(): String {
        return "(${String.format("%.3f", x)}, ${String.format("%.3f", y)})"
    }

    fun clone() = Vec(x, y)
}

operator fun Double.plus(vec: Vec) = Vec(this + vec.x, this + vec.y)
operator fun Double.minus(vec: Vec) = Vec(this - vec.x, this - vec.y)
operator fun Double.times(vec: Vec) = Vec(this * vec.x, this * vec.y)
operator fun Double.div(vec: Vec) = Vec(this / vec.x, this / vec.y)