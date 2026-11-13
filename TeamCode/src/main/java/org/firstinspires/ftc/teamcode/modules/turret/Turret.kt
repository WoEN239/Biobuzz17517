package org.firstinspires.ftc.teamcode.modules.turret

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.modules.driveTrain.GetOdometryConsumerEvent
import org.firstinspires.ftc.teamcode.modules.utils.GamepadListener
import org.firstinspires.ftc.teamcode.utils.motor.CachedMotor
import org.firstinspires.ftc.teamcode.utils.motor.CachedServo
import org.firstinspires.ftc.teamcode.utils.units.Ang
import org.firstinspires.ftc.teamcode.utils.units.Vec
import kotlin.math.PI
import kotlin.math.abs

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
    var TURRET_ROTATE_OFFSET = 0.0

    @JvmField
    var TURRET_CENTER_POS = -0.02067

    @JvmField
    var TURRET_MIN_ROTATE = Ang.ofDeg(-90.0)

    @JvmField
    var TURRET_MAX_ROTATE = Ang.ofDeg(90.0)

    @JvmField
    var TURRET_SERVO_MAX_ANGLE = PI * 1.5

    @JvmField
    var TURRET_SERVO_A = 9.0

    @JvmField
    var TURRET_SERVO_V_MAX = 4.0

    @JvmField
    var SHOOT_POINT = Vec(10.0 * 2.54 / 100.0, -12.0 * 2.54 / 100.0)

    @JvmField
    var MIN_SHOOT_POINT = Vec(-1.091, -0.346)

    @JvmField
    var MIN_PULLEY_VELOCITY = 1340.0

    @JvmField
    var MIN_TURRET_ANGLE = 0.69

    @JvmField
    var MAX_SHOOT_POINT = Vec(-1.512, -0.934)

    @JvmField
    var MAX_PULLEY_VELOCITY = 1420.0

    @JvmField
    var MAX_TURRET_ANGLE = 0.965

    @JvmField
    var MIN_TURRET_SERVO_ANGLE = 0.55

    @JvmField
    var MAX_TURRET_SERVO_ANGLE = 0.97

    @JvmField
    var PULLEY_VELOCITY_COMPENSATE_K = 0.0//2.1

    @JvmField
    var HEADING_LINEAR_VELOCITY_COMPENSATE_K = 0.0//0.71

    @JvmField
    var HEADING_ANGULAR_VELOCITY_COMPENSATE_K = 0.0
}

fun attachTurret(collector: Collector) {
    val motor = CachedMotor(collector.hardwareMap.get("turretMotor") as DcMotorEx)

    val turretAngleServo = CachedServo(collector.hardwareMap.get("turretAngleServo") as Servo)

    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    val minL = (TURRET_CONFIG.SHOOT_POINT * Vec(-1.0, 1.0) - TURRET_CONFIG.MIN_SHOOT_POINT).l()
    val maxL = (TURRET_CONFIG.SHOOT_POINT * Vec(-1.0, 1.0) - TURRET_CONFIG.MAX_SHOOT_POINT).l()

    val rawTurretRotateServo = collector.hardwareMap.get("turretRotateServo") as Servo

    (rawTurretRotateServo as ServoImplEx).pwmRange = PwmControl.PwmRange(500.0, 2500.0)

    val turretRotateServo = CachedServo(rawTurretRotateServo)

    val odometryConsumer = collector.eventBus(GetOdometryConsumerEvent()).consumer

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

    collector.updateEvent += {
        motor.setVelocityPIDFCoefficients(TURRET_CONFIG.REGULATOR.p, TURRET_CONFIG.REGULATOR.i, TURRET_CONFIG.REGULATOR.d, TURRET_CONFIG.REGULATOR.f)

//        motor.velocity = TURRET_CONFIG.PULLEY_VELOCITY
//
//        turretAngleServo.position = clamp(TURRET_CONFIG.TURRET_ANGLE, TURRET_CONFIG.MIN_TURRET_SERVO_ANGLE,
//            TURRET_CONFIG.MAX_TURRET_SERVO_ANGLE)

        val odometry = odometryConsumer()

        val turretPulleyErr = (TURRET_CONFIG.SHOOT_POINT * Vec(
            if (odometry.pos.x > 0.0) 1.0 else -1.0,
            1.0
        ) + (if (odometry.pos.x < 0.0) shootOffsetLeft else shootOffsetRight) - odometry.linearVel.turn(
            odometry.pos.angle()
        ) * TURRET_CONFIG.PULLEY_VELOCITY_COMPENSATE_K) - (odometry.pos.vec + Vec(
            TURRET_CONFIG.TURRET_CENTER_POS, 0.0
        ).turn(odometry.pos.angle.angle))


        val turretHeadingErr = (TURRET_CONFIG.SHOOT_POINT * Vec(
            if (odometry.pos.x > 0.0) 1.0 else -1.0,
            1.0
        ) + (if (odometry.pos.x < 0.0) shootOffsetLeft else shootOffsetRight) - odometry.linearVel.turn(
            odometry.pos.angle()
        ) * TURRET_CONFIG.HEADING_LINEAR_VELOCITY_COMPENSATE_K) - (odometry.pos.vec + Vec(
            TURRET_CONFIG.TURRET_CENTER_POS, 0.0
        ).turn(odometry.pos.angle.angle))

        turretRotateServo.position = clamp(
            (odometry.pos.angle - Ang(turretHeadingErr.rot() - odometry.angularVel * TURRET_CONFIG.HEADING_ANGULAR_VELOCITY_COMPENSATE_K) + PI).angle + TURRET_CONFIG.TURRET_ROTATE_OFFSET,
            TURRET_CONFIG.TURRET_MIN_ROTATE.angle, TURRET_CONFIG.TURRET_MAX_ROTATE.angle
        ) / TURRET_CONFIG.TURRET_SERVO_MAX_ANGLE + 0.478

        var l = turretPulleyErr.l()

        collector.telemetry.addData("turret raw l", l)

        l = clamp(l, minL, maxL)

        val k = (l - minL) / (maxL - minL)

        val targetPulleyVelocity =
            TURRET_CONFIG.MIN_PULLEY_VELOCITY + (TURRET_CONFIG.MAX_PULLEY_VELOCITY - TURRET_CONFIG.MIN_PULLEY_VELOCITY) * k
        val turretAngle =
            TURRET_CONFIG.MIN_TURRET_ANGLE + (TURRET_CONFIG.MAX_TURRET_ANGLE - TURRET_CONFIG.MIN_TURRET_ANGLE) * k

        motor.velocity = targetPulleyVelocity
        turretAngleServo.position = clamp(
            turretAngle, TURRET_CONFIG.MIN_TURRET_SERVO_ANGLE,
            TURRET_CONFIG.MAX_TURRET_SERVO_ANGLE
        )

        collector.telemetry.addData("target pulley velocity", TURRET_CONFIG.PULLEY_VELOCITY)
        collector.telemetry.addData("pulley velocity", motor.velocity)
    }
}