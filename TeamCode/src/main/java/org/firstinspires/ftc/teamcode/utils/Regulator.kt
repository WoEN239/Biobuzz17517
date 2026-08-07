package org.firstinspires.ftc.teamcode.utils

import androidx.core.math.MathUtils.clamp
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

data class RegParams(
    @JvmField var kP: Double = 0.0,
    @JvmField var kD: Double = 0.0,
    @JvmField var kI: Double = 0.0,
    @JvmField var kPow: Double = 0.0,
    @JvmField var kF: Double = 0.0,
    @JvmField var kG: Double = 0.0,
    @JvmField var kSG: Double = 0.0,
    @JvmField var limitU: Double = -1.0,
    @JvmField var resetI: Boolean = false
)

class Reg(val parameters: RegParams) {
    private val _deltaTime = ElapsedTime()

    private var _integral = 0.0
    private var _errOld = 0.0
    private var _oldKi = parameters.kI

    fun start() {
        _deltaTime.reset()
    }

    fun update(err: Double, target: Double, battery: Double): Double {
        val uP = err * parameters.kP

        val uD = (err - _errOld) / _deltaTime.seconds() * parameters.kD

        val uF = target * parameters.kF

        val uG = parameters.kG

        val uSG = parameters.kSG * target.sign

        val uPow = err.pow(2.0) * parameters.kPow * err.sign

        val uI = _integral * parameters.kI

        var u = uP + uI + uD + uF + uG + uSG + uPow

        if (err * _errOld < 0.0 && parameters.resetI)
            resetI()

        val limitU = parameters.limitU

        if (
            (limitU > 0.0 && u < limitU && u > -limitU) ||
            (limitU < 0.0 && u < battery && u > -battery) ||
            (err * u < 0.0)
        )
            _integral += err * _deltaTime.seconds()

        _deltaTime.reset()

        if (abs(_oldKi - parameters.kI) > 0.00001)
            resetI()

        _oldKi = parameters.kI

        _errOld = err

        if (limitU >= 0.0)
            u = clamp(u, -limitU, limitU)

        return u
    }

    fun resetI() {
        _integral = 0.0
    }

    fun update(err: Double) = update(err, 0.0, 12.0)
}