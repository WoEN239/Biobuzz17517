package org.firstinspires.ftc.teamcode.modules.turret

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.collector.Collector
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
    var REGULATOR = TurretRegulatorParams(45.0, 2.0, 0.0, 13.5)

    @JvmField
    var VELOCITY = 0.0
}

fun attachTurret(collector: Collector) {
    val motor = collector.hardwareMap.get("turretMotor") as DcMotorEx

    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

    motor.mode = DcMotor.RunMode.RUN_USING_ENCODER

    motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

    collector.updateEvent += {
        motor.setVelocityPIDFCoefficients(TURRET_CONFIG.REGULATOR.p, TURRET_CONFIG.REGULATOR.i, TURRET_CONFIG.REGULATOR.d, TURRET_CONFIG.REGULATOR.f)

        motor.velocity = TURRET_CONFIG.VELOCITY

        collector.telemetry.addData("target velocity", TURRET_CONFIG.VELOCITY)
        collector.telemetry.addData("velocity", motor.velocity)
    }
}