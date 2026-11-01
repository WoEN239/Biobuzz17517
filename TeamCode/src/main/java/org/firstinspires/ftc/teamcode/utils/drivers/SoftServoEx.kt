package org.firstinspires.ftc.teamcode.utils.drivers

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.modules.utils.Telemetry
import org.firstinspires.ftc.teamcode.utils.motor.CachedServo
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

@Config
internal object SOFT_SERVO_EX_CONFIG {
    @JvmField
    var SERVO_A = 0.1

    @JvmField
    var SERVO_V_MAX = 3.5
}

class SoftServoEx(
    servoName: String,
    hardwareMap: HardwareMap,
    private val _startPosition: Double = 0.0,
    var a: Double = SOFT_SERVO_CONFIG.SERVO_A,
    var vMax: Double = SOFT_SERVO_CONFIG.SERVO_V_MAX
) {
    private val _rawServo = hardwareMap.get(servoName) as Servo
    private val _servo = CachedServo(_rawServo)

    private val _deltaTime = ElapsedTime()

    var targetPosition: Double = _startPosition

    var currentPosition
        get() = _servo.position
        private set(value) {
            _servo.position = clamp(value, 0.0, 1.0)
        }

    private var _velocity = 0.0

    fun start() {
        currentPosition = _startPosition
        targetPosition = _startPosition

        (_rawServo as ServoImplEx).pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    fun update() {
        val dt = _deltaTime.seconds()
        _deltaTime.reset()

        val eps = 1e-6
        val e = targetPosition - currentPosition

        if (abs(e) < eps && abs(_velocity) < eps)
            _velocity = 0.0
        else {
            val s = sign(e)
            val vAllow = sqrt(2.0 * a * abs(e))
            val vDes = s * minOf(vMax, vAllow)

            var dv = vDes - _velocity
            dv = if (dv > 0) minOf(dv, a * dt) else maxOf(dv, -a * dt)

            var vCmd = _velocity + dv

            if (abs(vCmd) > eps) {
                val nextPos = currentPosition + vCmd * dt
                if ((targetPosition - nextPos) * e < 0.0) vCmd = e / dt
            }

            _velocity = vCmd
        }

        currentPosition += _velocity * dt
    }

    fun attach(collector: Collector): SoftServoEx {
        collector.startEvent += {
            start()
        }

        collector.updateEvent += {
            update()
        }

        return this
    }
}