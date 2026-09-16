package org.firstinspires.ftc.teamcode.modules.driveTrain

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.collector.GameColor
import org.firstinspires.ftc.teamcode.collector.RunMode
import org.firstinspires.ftc.teamcode.collector.Settings
import org.firstinspires.ftc.teamcode.modules.utils.IGamepadListener
import org.firstinspires.ftc.teamcode.utils.Reg
import org.firstinspires.ftc.teamcode.utils.RegParams
import org.firstinspires.ftc.teamcode.utils.motor.MotorOnly
import org.firstinspires.ftc.teamcode.utils.units.Vec
import kotlin.math.PI
import kotlin.math.absoluteValue

@Config
internal object DRIVE_TRAIN_CONFIG {
    @JvmField
    var X_REGULATOR = RegParams()

    @JvmField
    var Y_REGULATOR = RegParams()

    @JvmField
    var H_REGULATOR = RegParams()
}

class SetDriveVelEvent(val linearVel: Vec, val angularVel: Double)

fun attachDriveTrain(collector: Collector) {
    val leftForwardMotor = MotorOnly(
        collector.hardwareMap
            .get("leftForwardDrive") as DcMotorEx
    )
    val leftBackMotor = MotorOnly(
        collector.hardwareMap
            .get("leftBackDrive") as DcMotorEx
    )
    val rightForwardMotor = MotorOnly(
        collector.hardwareMap
            .get("rightForwardDrive") as DcMotorEx
    )
    val rightBackMotor = MotorOnly(
        collector.hardwareMap
            .get("rightBackDrive") as DcMotorEx
    )

    leftForwardMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    leftBackMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    rightForwardMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    rightBackMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    val odometryConsumer = collector.eventBus(GetOdometryConsumerEvent()).consumer

    var linearPower = Vec.ZERO
    var angularPower = 0.0

    var targetLinearVelocity = Vec.ZERO
    var targetAngularVelocity = 0.0

    val xRegulator = Reg(DRIVE_TRAIN_CONFIG.X_REGULATOR)
    val yRegulator = Reg(DRIVE_TRAIN_CONFIG.Y_REGULATOR)
    val hRegulator = Reg(DRIVE_TRAIN_CONFIG.H_REGULATOR)

    collector.eventBus.sub(SetDriveVelEvent::class) {
        targetLinearVelocity = it.linearVel
        targetAngularVelocity = it.angularVel
    }

    collector.gamepad.addGamepad1Listener(object : IGamepadListener {
        override fun update(gamepadData: Gamepad) {
            val pos = odometryConsumer().pos

            linearPower =
                Vec(
                    -gamepadData.left_stick_y.toDouble(),
                    -gamepadData.left_stick_x.toDouble()
                ).turn(
                    -pos.angle() +
                            (if (Settings.orientation.color == GameColor.BLUE) -PI else PI) / 2.0
                )
            angularPower = -gamepadData.right_stick_x.toDouble()

            linearPower *= Vec(
                linearPower.x.absoluteValue,
                linearPower.y.absoluteValue
            )
            angularPower *= angularPower.absoluteValue
        }
    })

    fun setPowers(
        leftForwardPower: Double,
        leftBackPower: Double,
        rightForwardPower: Double,
        rightBackPower: Double
    ) {
        var leftForwardPower = leftForwardPower
        var leftBackPower = leftBackPower
        var rightForwardPower = rightForwardPower
        var rightBackPower = rightBackPower

        var absoluteMaximum = leftForwardPower.absoluteValue

        if (absoluteMaximum < leftBackPower.absoluteValue)
            absoluteMaximum = leftBackPower.absoluteValue

        if (absoluteMaximum < rightForwardPower.absoluteValue)
            absoluteMaximum = rightForwardPower.absoluteValue

        if (absoluteMaximum < rightBackPower.absoluteValue)
            absoluteMaximum = rightBackPower.absoluteValue

        if (absoluteMaximum > 1.0) {
            leftForwardPower /= absoluteMaximum
            leftBackPower /= absoluteMaximum
            rightForwardPower /= absoluteMaximum
            rightBackPower /= absoluteMaximum
        }

        leftForwardMotor.power = leftForwardPower
        leftBackMotor.power = leftBackPower
        rightForwardMotor.power = rightForwardPower
        rightBackMotor.power = rightBackPower
    }

    collector.startEvent += {
        xRegulator.start()
        yRegulator.start()
        hRegulator.start()
    }

    collector.updateEvent += if (collector.runMode == RunMode.MANUAL) {
        {
            setPowers(
                linearPower.x - linearPower.y - angularPower,
                linearPower.x + linearPower.y - angularPower,
                linearPower.x + linearPower.y + angularPower,
                linearPower.x - linearPower.y + angularPower
            )
        }
    } else {
        {
            val odometry = odometryConsumer()

            val velocityErr = targetLinearVelocity - odometry.linearVel

            linearPower = Vec(
                xRegulator.update(
                    velocityErr.x,
                    targetLinearVelocity.x,
                    collector.battery.currentVoltage
                ),
                yRegulator.update(
                    velocityErr.y,
                    targetLinearVelocity.y,
                    collector.battery.currentVoltage
                )
            )

            angularPower = hRegulator.update(
                targetAngularVelocity - odometry.angularVel,
                targetAngularVelocity,
                collector.battery.currentVoltage
            )

            setPowers(
                collector.battery.voltageToPower(linearPower.x - linearPower.y - angularPower),
                collector.battery.voltageToPower(linearPower.x + linearPower.y - angularPower),
                collector.battery.voltageToPower(linearPower.x + linearPower.y + angularPower),
                collector.battery.voltageToPower(linearPower.x - linearPower.y + angularPower)
            )
        }
    }
}