package org.firstinspires.ftc.teamcode.modules.turret

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.modules.driveTrain.GetOdometryConsumerEvent
import org.firstinspires.ftc.teamcode.modules.utils.GamepadListener
import org.firstinspires.ftc.teamcode.utils.motor.CachedMotor
import org.firstinspires.ftc.teamcode.utils.motor.CachedServo
import org.firstinspires.ftc.teamcode.utils.units.Ang
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Vec
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

data class TurretRegulatorParams(
    @JvmField var p: Double,
    @JvmField var i: Double,
    @JvmField var d: Double,
    @JvmField var f: Double
) {
    override fun equals(other: Any?): Boolean {
        if (other == null)
            return false

        if (other !is TurretRegulatorParams)
            return false

        if (abs(other.p - p) < 0.00001 && abs(other.i - i) < 0.00001 && abs(other.d - d) < 0.00001 && abs(
                other.f - f
            ) < 0.00001
        )
            return true

        return false
    }

    override fun hashCode(): Int {
        var result = p.hashCode()
        result = 31 * result + i.hashCode()
        result = 31 * result + d.hashCode()
        result = 31 * result + f.hashCode()
        return result
    }

    override fun toString(): String {
        return "($p, $i, $d, $f)"
    }
}

@Config
internal object TURRET_CONFIG {
    @JvmField
    var REGULATOR = TurretRegulatorParams(250.0, 0.0, 0.0, 17.4)

    @JvmField
    var PULLEY_VELOCITY = 0.0

    @JvmField
    var TURRET_ANGLE = 0.5

    @JvmField
    var CENTER_POS = -0.02067

    @JvmField
    var MIN_ROTATE = Ang.ofDeg(-90.0)

    @JvmField
    var MAX_ROTATE = Ang.ofDeg(90.0)

    @JvmField
    var ROTATE_SERVO_MAX_ANGLE = PI * 1.5

    @JvmField
    var SHOOT_POINT = Vec(11.0 * 2.54 / 100.0, -12.0 * 2.54 / 100.0 - 0.05)

    @JvmField
    var SERVO_MIN_ANGLE = 0.55

    @JvmField
    var SERVO_MAX_ANGLE = 0.96

    @JvmField
    var PULLEY_AT_TARGET_SENS = 100.0

    @JvmField
    var TURRET_POINTS = arrayListOf(
//        TurretPoint(1.916, 1530.0, 0.96,)
        TurretPoint(0.84, 1360.0, 0.72, 1.0),
        TurretPoint(1.207, 1400.0, 0.87, 1.0),
//        TurretPoint(1.45, 1420.0, 0.88, 1.0),
        TurretPoint(2.148, 1580.0, 0.96, 1.0)
    )

    @JvmField
    var PULLEY_VELOCITY_COMPENSATE_K = 0.0//1.0

    @JvmField
    var CALCULATE_STEPS = 4

    @JvmField
    var HEADING_ANGULAR_VELOCITY_COMPENSATE_K = 0.12

    @JvmField
    var HEADING_FEED_FORWARD = 10.0

    @JvmField
    var PULLEY_FEED_FORWARD = 10.0

    @JvmField
    var HIVE_FACE_POINT = Vec(0.53, -12.0 * 2.54 / 100.0 - 0.05)

    @JvmField
    var HIVE_FACE_OFFSET = 0.17
}

class TurretPoint(
    @JvmField val l: Double,
    @JvmField val velocity: Double,
    @JvmField val angle: Double,
    @JvmField val time: Double
)

class GetTurretAtTargetConsumer(var consumer: () -> Boolean = { false })

fun attachTurret(collector: Collector) {
    val motor = CachedMotor(collector.hardwareMap.get("turretMotor") as DcMotorEx)

    val turretAngleServo = CachedServo(collector.hardwareMap.get("turretAngleServo") as Servo)

    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    val rawTurretRotateServo = collector.hardwareMap.get("turretRotateServo") as Servo

    (rawTurretRotateServo as ServoImplEx).pwmRange = PwmControl.PwmRange(500.0, 2500.0)

    val turretRotateServo = CachedServo(rawTurretRotateServo)

    val odometryConsumer = collector.eventBus(GetOdometryConsumerEvent()).consumer

    var turretAtTarget = false
    var turretOldRotate: Ang? = null

    var turretOldL: Double? = null

    val deltaTime = ElapsedTime()

    collector.eventBus.sub(GetTurretAtTargetConsumer::class) {
        it.consumer = { turretAtTarget }
    }

    motor.setVelocityPIDFCoefficients(
        TURRET_CONFIG.REGULATOR.p,
        TURRET_CONFIG.REGULATOR.i,
        TURRET_CONFIG.REGULATOR.d,
        TURRET_CONFIG.REGULATOR.f
    )

    var shootOffsetLeft = Vec.ZERO

    var shootOffsetRight = Vec.ZERO

    collector.gamepad.addGamepad2Listener(GamepadListener({ it.dpadLeftWasPressed() }, {
        shootOffsetLeft += Vec(3.0 * 2.54 / 100.0, 0.0)
    }))

    collector.gamepad.addGamepad2Listener(GamepadListener({ it.dpadRightWasPressed() }, {
        shootOffsetLeft += Vec(-3.0 * 2.54 / 100.0, 0.0)
    }))

    collector.gamepad.addGamepad2Listener(GamepadListener({ it.dpadUpWasPressed() }, {
        shootOffsetLeft += Vec(0.0, 3.0 * 2.54 / 100.0)
    }))

    collector.gamepad.addGamepad2Listener(GamepadListener({ it.dpadDownWasPressed() }, {
        shootOffsetLeft += Vec(0.0, -3.0 * 2.54 / 100.0)
    }))

    collector.gamepad.addGamepad2Listener(GamepadListener({ it.circleWasPressed() }, {
        shootOffsetRight += Vec(3.0 * 2.54 / 100.0, 0.0)
    }))

    collector.gamepad.addGamepad2Listener(GamepadListener({ it.squareWasPressed() }, {
        shootOffsetRight += Vec(-3.0 * 2.54 / 100.0, 0.0)
    }))

    collector.gamepad.addGamepad2Listener(GamepadListener({ it.triangleWasPressed() }, {
        shootOffsetRight += Vec(0.0, 3.0 * 2.54 / 100.0)
    }))

    collector.gamepad.addGamepad2Listener(GamepadListener({ it.crossWasPressed() }, {
        shootOffsetRight += Vec(0.0, -3.0 * 2.54 / 100.0)
    }))

    collector.gamepad.addGamepad1Listener(GamepadListener({ it.dpadLeftWasReleased() }, {
        shootOffsetLeft = Vec.ZERO
        shootOffsetRight = Vec.ZERO
    }))

    collector.startEvent += {
        deltaTime.reset()
    }

    collector.updateEvent += {
//        motor.setVelocityPIDFCoefficients(
//            TURRET_CONFIG.REGULATOR.p,
//            TURRET_CONFIG.REGULATOR.i,
//            TURRET_CONFIG.REGULATOR.d,
//            TURRET_CONFIG.REGULATOR.f
//        )

//        motor.velocity = TURRET_CONFIG.PULLEY_VELOCITY

//        turretAngleServo.position = clamp(
//            TURRET_CONFIG.TURRET_ANGLE, TURRET_CONFIG.SERVO_MIN_ANGLE,
//            TURRET_CONFIG.SERVO_MAX_ANGLE
//        )

        val dt = deltaTime.seconds()
        deltaTime.reset()

        val odometry = odometryConsumer()

        val turretPoint = odometry.pos.vec + Vec(TURRET_CONFIG.CENTER_POS, 0.0)
            .turn(odometry.pos.angle.angle)

        val shootGamepadOffset = if (turretPoint.x < 0.0) shootOffsetLeft else shootOffsetRight
        val shootSideMultiplier = Vec(if (turretPoint.x > 0.0) 1.0 else -1.0, 1.0)

        val aimPoint = TURRET_CONFIG.SHOOT_POINT * shootSideMultiplier + shootGamepadOffset

        val hiveFaceOffset = Vec(
            0.0, if (turretPoint.y > aimPoint.y)
                TURRET_CONFIG.HIVE_FACE_OFFSET else -TURRET_CONFIG.HIVE_FACE_OFFSET
        )

        val hiveFaceCenter = TURRET_CONFIG.HIVE_FACE_POINT * shootSideMultiplier + shootGamepadOffset

        val hiveFacePoint = hiveFaceCenter + hiveFaceOffset

        collector.telemetry.drawCircle(aimPoint, 0.05, Color.RED)
        collector.telemetry.drawCircle(hiveFacePoint, 0.05, Color.BLACK)
        collector.telemetry.drawCircle(turretPoint, 0.05, Color.GREEN)

        val alpha =
            (Ang((turretPoint - hiveFacePoint).rot()) - Ang((turretPoint - aimPoint).rot())) *
                    (if (turretPoint.y > aimPoint.y) 1.0 else -1.0) * -shootSideMultiplier.x

        val shootPoint = (if (alpha.angle > 0.0) {
            { aimPoint }
        } else {
            {
                fun getIntersection(p1: Vec, p2: Vec, p3: Vec, p4: Vec): Vec {
                    var delta = (p2.x - p1.x) * (p4.y - p3.y) - (p2.y - p1.y) * (p4.x - p3.x)

                    if (delta.absoluteValue < 0.0001)
                        delta += 0.0001

                    val t = ((p3.x - p1.x) * (p4.y - p3.y) - (p3.y - p1.y) * (p4.x - p3.x)) / delta

                    return Vec(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y))
                }

                val i1 =
                    getIntersection(turretPoint, hiveFacePoint, aimPoint, aimPoint - hiveFaceOffset)
                val i2 = getIntersection(
                    turretPoint,
                    hiveFacePoint,
                    aimPoint - hiveFaceOffset,
                    hiveFaceCenter - hiveFaceOffset
                )

                val p =
                    if ((i1.x >= i2.x && turretPoint.x < 0) || (i1.x <= i2.x && turretPoint.x > 0)) i2 else i1

                if (turretPoint.x > 0.0)
                    Vec(
                        clamp(p.x, aimPoint.x, hiveFaceCenter.x),
                        clamp(
                            p.y,
                            hiveFaceCenter.y - TURRET_CONFIG.HIVE_FACE_OFFSET,
                            hiveFaceCenter.y + TURRET_CONFIG.HIVE_FACE_OFFSET
                        )
                    )
                else
                    Vec(
                        clamp(p.x, hiveFaceCenter.x, aimPoint.x),
                        clamp(
                            p.y,
                            hiveFaceCenter.y - TURRET_CONFIG.HIVE_FACE_OFFSET,
                            hiveFaceCenter.y + TURRET_CONFIG.HIVE_FACE_OFFSET
                        )
                    )
            }
        })()

        collector.telemetry.drawCircle(shootPoint, 0.05, Color.GREEN)

        val globalRobotVelocity = odometry.linearVel.turn(odometry.pos.angle())

        val turretPulleyErr = shootPoint - turretPoint

        fun calcPont(l: Double): TurretPoint {
            val points = TURRET_CONFIG.TURRET_POINTS

            var firstIndex = 0

            for (i in 1..<points.size - 1) {
                if (points[i].l > l)
                    break

                firstIndex = i
            }

            val min = points[firstIndex]
            val max = points[firstIndex + 1]

            val k = (l - min.l) / (max.l - min.l)

            return TurretPoint(
                l,
                min.velocity + (max.velocity - min.velocity) * k,
                min.angle + (max.angle - min.angle) * k,
                min.time + (max.time - min.time)
            )
        }

        val rawL = turretPulleyErr.l()
        val l =
            clamp(rawL, TURRET_CONFIG.TURRET_POINTS.first().l, TURRET_CONFIG.TURRET_POINTS.last().l)

        val dream = calcPont(l)

        val dreamShootPoint =
            globalRobotVelocity * -TURRET_CONFIG.PULLEY_VELOCITY_COMPENSATE_K * dream.time + shootPoint

        val dreamErr = dreamShootPoint - turretPoint

        val calc = calcPont(dreamErr.l())

        val targetPulleyVelocity = calc.velocity
        val targetTurretAngle = calc.angle

        turretAngleServo.position = clamp(
            targetTurretAngle,
            TURRET_CONFIG.SERVO_MIN_ANGLE,
            TURRET_CONFIG.SERVO_MAX_ANGLE
        )

        motor.velocity = targetPulleyVelocity

        turretAtTarget =
            (motor.velocity - targetPulleyVelocity).absoluteValue < TURRET_CONFIG.PULLEY_AT_TARGET_SENS

        val turretRotate = Ang(dreamErr.rot())

        turretRotateServo.position = clamp(
            (odometry.pos.angle - turretRotate + PI).angle +
                    odometry.angularVel * TURRET_CONFIG.HEADING_ANGULAR_VELOCITY_COMPENSATE_K +
                    if (turretOldRotate != null) (turretRotate - turretOldRotate!!).angle * -TURRET_CONFIG.HEADING_FEED_FORWARD * dt else 0.0,
            TURRET_CONFIG.MIN_ROTATE.angle, TURRET_CONFIG.MAX_ROTATE.angle
        ) / TURRET_CONFIG.ROTATE_SERVO_MAX_ANGLE + 0.56

        turretOldRotate = turretRotate

        collector.telemetry.addData("raw turret l", rawL)
        collector.telemetry.addData("target pulley velocity", TURRET_CONFIG.PULLEY_VELOCITY)
        collector.telemetry.addData("pulley velocity", motor.velocity)
    }
}