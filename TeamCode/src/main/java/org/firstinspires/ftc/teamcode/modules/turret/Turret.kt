package org.firstinspires.ftc.teamcode.modules.turret

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.modules.driveTrain.GetOdometryConsumerEvent
import org.firstinspires.ftc.teamcode.utils.drivers.SoftServoEx
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
    var REGULATOR = TurretRegulatorParams(55.0, 0.1, 0.0, 13.5)

    @JvmField
    var VELOCITY = 0.0

    @JvmField
    var TURRET_CENTER_POS = -0.02067

    @JvmField
    var TURRET_MIN_ANGLE = Ang.ofDeg(-90.0)

    @JvmField
    var TURRET_MAX_ANGLE = Ang.ofDeg(90.0)

    @JvmField
    var TURRET_SERVO_MAX_ANGLE = PI * 1.5

    @JvmField
    var TURRET_SERVO_A = 9.0

    @JvmField
    var TURRET_SERVO_V_MAX = 4.0
}

fun attachTurret(collector: Collector) {
    val motor = collector.hardwareMap.get("turretMotor") as DcMotorEx

    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    val turretRotateServo = SoftServoEx("turretRotateServo", collector.hardwareMap, 0.5, a = TURRET_CONFIG.TURRET_SERVO_A, vMax = TURRET_CONFIG.TURRET_SERVO_V_MAX).attach(collector)

    val odometryConsumer = collector.eventBus(GetOdometryConsumerEvent()).consumer

    motor.setVelocityPIDFCoefficients(TURRET_CONFIG.REGULATOR.p, TURRET_CONFIG.REGULATOR.i, TURRET_CONFIG.REGULATOR.d, TURRET_CONFIG.REGULATOR.f)

    collector.updateEvent += {
//        motor.setVelocityPIDFCoefficients(TURRET_CONFIG.REGULATOR.p, TURRET_CONFIG.REGULATOR.i, TURRET_CONFIG.REGULATOR.d, TURRET_CONFIG.REGULATOR.f)

        motor.velocity = TURRET_CONFIG.VELOCITY

        collector.telemetry.addData("target velocity", TURRET_CONFIG.VELOCITY)
        collector.telemetry.addData("velocity", motor.velocity)

        val odometry = odometryConsumer()

        turretRotateServo.targetPosition = clamp((odometry.pos.angle - Ang((-1.0 * (odometry.pos.vec + Vec(
            TURRET_CONFIG.TURRET_CENTER_POS, 0.0).turn(odometry.pos.angle.angle))).rot()) + PI).angle,
            TURRET_CONFIG.TURRET_MIN_ANGLE.angle, TURRET_CONFIG.TURRET_MAX_ANGLE.angle) / TURRET_CONFIG.TURRET_SERVO_MAX_ANGLE + 0.5

        collector.telemetry.addData("turret target rotate", turretRotateServo.targetPosition)
        collector.telemetry.addData("turret current rotate", turretRotateServo.currentPosition)

        turretRotateServo.a = TURRET_CONFIG.TURRET_SERVO_A
        turretRotateServo.vMax = TURRET_CONFIG.TURRET_SERVO_V_MAX
    }
}