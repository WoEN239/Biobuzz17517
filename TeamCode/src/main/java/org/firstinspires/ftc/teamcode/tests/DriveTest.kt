package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.utils.motor.MotorOnly
import org.firstinspires.ftc.teamcode.utils.units.Vec
import kotlin.math.absoluteValue
import kotlin.math.max

@TeleOp(group = "tests")
class DriveTest : LinearOpMode() {
    override fun runOpMode() {
        val leftForwardMotor = MotorOnly(hardwareMap.get("leftForwardMotor") as DcMotorEx)
        val leftBackMotor = MotorOnly(hardwareMap.get("leftBackMotor") as DcMotorEx)
        val rightForwardMotor = MotorOnly(hardwareMap.get("rightForwardMotor") as DcMotorEx)
        val rightBackMotor = MotorOnly(hardwareMap.get("rightBackMotor") as DcMotorEx)

        val transferMotor = MotorOnly(hardwareMap.get("transferMotor") as DcMotorEx)

        transferMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        transferMotor.direction = DcMotorSimple.Direction.REVERSE

        val brushMotor = MotorOnly(hardwareMap.get("brushMotor") as DcMotorEx)

        brushMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        leftForwardMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBackMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightForwardMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBackMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        leftForwardMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()
        resetRuntime()

        while (opModeIsActive()) {
            val dir = Vec(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble()
            )
            val rot = -gamepad1.right_stick_x

            var leftForwardPower = dir.x - dir.y - rot
            var leftBackPower = dir.x + dir.y - rot
            var rightForwardPower = dir.x + dir.y + rot
            var rightBackPower = dir.x - dir.y + rot

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

            brushMotor.power = if(gamepad1.right_bumper) 1.0 else if(gamepad1.left_bumper) -1.0 else 0.0

            transferMotor.power = brushMotor.power
        }
    }
}