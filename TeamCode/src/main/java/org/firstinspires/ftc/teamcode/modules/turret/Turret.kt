package org.firstinspires.ftc.teamcode.modules.turret

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.modules.driveTrain.GetOdometryConsumerEvent
import org.firstinspires.ftc.teamcode.utils.drivers.SoftServoEx
import org.firstinspires.ftc.teamcode.utils.motor.CachedMotor
import org.firstinspires.ftc.teamcode.utils.motor.CachedServo
import org.firstinspires.ftc.teamcode.utils.units.Ang
import org.firstinspires.ftc.teamcode.utils.units.Vec
import org.firstinspires.ftc.teamcode.utils.units.times
import kotlin.math.PI
import kotlin.math.abs

data class TurretRegulatorParams(
    @JvmField var p: Double,
    @JvmField var i: Double,
    @JvmField var d: Double,
    @JvmField var f: Double
) {
    override fun equals(other: Any?): Boolean {
        if(other == null)
            return false

        if(other !is TurretRegulatorParams)
            return false

        if(abs(other.p - p) < 0.00001 && abs(other.i - i) < 0.00001 && abs(other.d - d) < 0.00001 && abs(other.f - f) < 0.00001)
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
    var REGULATOR = TurretRegulatorParams(52.0, 0.0, 0.0, 13.2)

    @JvmField
    var PULLEY_VELOCITY = 0.0

    @JvmField
    var TURRET_ANGLE = 0.5

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
    var SHOOT_POINT = Vec(20.0 * 2.54 / 100.0, -12.0 * 2.54 / 100.0)

    @JvmField
    var MIN_SHOOT_DISTANCE = 0.43

    @JvmField
    var MAX_SHOOT_DISTANCE = 1.59

    @JvmField
    var MIN_PULLEY_VELOCITY = 1660.0

    @JvmField
    var MAX_PULLEY_VELOCITY = 2065.0

    @JvmField
    var MIN_TURRET_ANGLE = 0.74

    @JvmField
    var MAX_TURRET_ANGLE = 1.07

    @JvmField
    var MIN_TURRET_ANGLE_SERVO = 0.7

    @JvmField
    var MAX_TURRET_ANGLE_SERVO = 0.945
}

fun attachTurret(collector: Collector) {
    val motor = CachedMotor(collector.hardwareMap.get("turretMotor") as DcMotorEx)

    val turretAngleServo = CachedServo(collector.hardwareMap.get("turretAngleServo") as Servo)

    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    val turretRotateServo = SoftServoEx("turretRotateServo", collector.hardwareMap, 0.5, a = TURRET_CONFIG.TURRET_SERVO_A, vMax = TURRET_CONFIG.TURRET_SERVO_V_MAX).attach(collector)

    val odometryConsumer = collector.eventBus(GetOdometryConsumerEvent()).consumer

    motor.setVelocityPIDFCoefficients(TURRET_CONFIG.REGULATOR.p, TURRET_CONFIG.REGULATOR.i, TURRET_CONFIG.REGULATOR.d, TURRET_CONFIG.REGULATOR.f)

    collector.updateEvent += {
//        motor.setVelocityPIDFCoefficients(TURRET_CONFIG.REGULATOR.p, TURRET_CONFIG.REGULATOR.i, TURRET_CONFIG.REGULATOR.d, TURRET_CONFIG.REGULATOR.f)

//        motor.velocity = TURRET_CONFIG.PULLEY_VELOCITY

//        turretAngleServo.position = clamp(TURRET_CONFIG.TURRET_ANGLE, TURRET_CONFIG.MIN_TURRET_ANGLE_SERVO,
//            TURRET_CONFIG.MAX_TURRET_ANGLE_SERVO)

        val odometry = odometryConsumer()

        val turretErr = TURRET_CONFIG.SHOOT_POINT * Vec(if(odometry.pos.x > 0.0) 1.0 else -1.0, 1.0) - (odometry.pos.vec + Vec(
            TURRET_CONFIG.TURRET_CENTER_POS, 0.0).turn(odometry.pos.angle.angle))

        turretRotateServo.targetPosition = clamp((odometry.pos.angle - Ang(turretErr.rot()) + PI).angle,
            TURRET_CONFIG.TURRET_MIN_ROTATE.angle, TURRET_CONFIG.TURRET_MAX_ROTATE.angle) / TURRET_CONFIG.TURRET_SERVO_MAX_ANGLE + 0.5

        var l = turretErr.l()

        collector.telemetry.addData("turret raw l", l)

        l = clamp(l, TURRET_CONFIG.MIN_SHOOT_DISTANCE, TURRET_CONFIG.MAX_SHOOT_DISTANCE)

        val k = (l - TURRET_CONFIG.MIN_SHOOT_DISTANCE) / (TURRET_CONFIG.MAX_SHOOT_DISTANCE - TURRET_CONFIG.MIN_SHOOT_DISTANCE)

        val targetPulleyVelocity = TURRET_CONFIG.MIN_PULLEY_VELOCITY + (TURRET_CONFIG.MAX_PULLEY_VELOCITY - TURRET_CONFIG.MIN_PULLEY_VELOCITY) * k
        val turretAngle = TURRET_CONFIG.MIN_TURRET_ANGLE + (TURRET_CONFIG.MAX_TURRET_ANGLE - TURRET_CONFIG.MIN_TURRET_ANGLE) * k

        motor.velocity = targetPulleyVelocity
        turretAngleServo.position = clamp(turretAngle, TURRET_CONFIG.MIN_TURRET_ANGLE_SERVO,
            TURRET_CONFIG.MAX_TURRET_ANGLE_SERVO)

        collector.telemetry.addData("turret target rotate", turretRotateServo.targetPosition)
        collector.telemetry.addData("turret current rotate", turretRotateServo.currentPosition)


        collector.telemetry.addData("target pulley velocity", targetPulleyVelocity)
        collector.telemetry.addData("pulley velocity", motor.velocity)

        turretRotateServo.a = TURRET_CONFIG.TURRET_SERVO_A
        turretRotateServo.vMax = TURRET_CONFIG.TURRET_SERVO_V_MAX
    }
}