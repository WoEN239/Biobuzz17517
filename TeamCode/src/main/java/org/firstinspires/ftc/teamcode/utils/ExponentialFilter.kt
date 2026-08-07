package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.util.ElapsedTime

class ExponentialFilter(var k: Double) {
    private val _deltaTime = ElapsedTime()

    fun start() {
        _deltaTime.reset()
    }

    fun updateRaw(value: Double, delta: Double): Double {
        val result = value + delta * (_deltaTime.seconds() / (k + _deltaTime.seconds()))

        _deltaTime.reset()

        return result
    }

    fun update(val1: Double, val2: Double): Double {
        return updateRaw(val1, val2 - val1)
    }
}