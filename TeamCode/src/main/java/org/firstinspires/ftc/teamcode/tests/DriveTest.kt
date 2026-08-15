package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.utils.motor.MotorOnly
import org.firstinspires.ftc.teamcode.utils.units.Vec
import kotlin.math.absoluteValue
import kotlin.math.max

@TeleOp(group = "tests")
class DriveTest : LinearOpMode() {
    override fun runOpMode() {
        val leftForwardMotor = MotorOnly(hardwareMap.get("leftForwardDrive") as DcMotorEx)
        val leftBackMotor = MotorOnly(hardwareMap.get("leftBackDrive") as DcMotorEx)
        val rightForwardMotor = MotorOnly(hardwareMap.get("rightForwardDrive") as DcMotorEx)
        val rightBackMotor = MotorOnly(hardwareMap.get("rightBackDrive") as DcMotorEx)

        leftForwardMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBackMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightForwardMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBackMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

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
        }
    }
}