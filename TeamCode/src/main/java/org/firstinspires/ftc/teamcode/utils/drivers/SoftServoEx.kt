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
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

@Config
internal object SOFT_SERVO_EX_CONFIG {
    @JvmField
    var SERVO_A = 5.0

    @JvmField
    var SERVO_V_MAX = 10.0

    @JvmField
    var S_TOLERANCE = 0.001
}

class SoftServoEx(
    servoName: String,
    hardwareMap: HardwareMap,
    private val _startPosition: Double = 0.0,
    var a: Double = SOFT_SERVO_CONFIG.SERVO_A,
    var vMax: Double = SOFT_SERVO_CONFIG.SERVO_V_MAX
) {
    private val _servo = hardwareMap.get(servoName) as Servo

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

        (_servo as ServoImplEx).pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    fun update(telemetry: MultipleTelemetry) {
        val ds = _deltaTime.seconds()
        _deltaTime.reset()

        var S = targetPosition - currentPosition
        val sS = S.sign
        S = S.absoluteValue

        telemetry.addData("target", targetPosition)
        telemetry.addData("velocity", _velocity)

        if (S < SOFT_SERVO_EX_CONFIG.S_TOLERANCE)
            _velocity = 0.0
        else {
            val t1 = (vMax * sS - _velocity).absoluteValue / a
            val S1 = t1 * (if(_velocity.sign != sS) t1 * a else (t1 * a / 2.0 + _velocity.absoluteValue))

            val S3 = (vMax / a).pow(2) * a / 2.0

            if (S1 + S3 < S) {
                _velocity += a * ds * sS
                telemetry.addLine("state 1")
            } else {
                val S3s = (_velocity / a).pow(2) * a / 2.0

                if (S < S3s || abs(S - S3s) < SOFT_SERVO_EX_CONFIG.S_TOLERANCE) {
                    _velocity -= a * ds * sS

                    telemetry.addLine("state 2")
                }
                else {
                    _velocity += a * ds * sS

                    telemetry.addLine("state 3")
                }
            }
        }

        _velocity = clamp(_velocity, -vMax, vMax)
        currentPosition += _velocity * ds
    }

    fun attach(collector: Collector): SoftServoEx {
        collector.startEvent += {
            start()
        }

//        collector.updateEvent += {
//            update()
//        }

        return this
    }
}