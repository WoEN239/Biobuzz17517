package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.utils.motor.MotorOnly

@TeleOp(group = "tests")
class DriveTrainDirectionTest: LinearOpMode() {
    override fun runOpMode() {
        val leftForwardMotor = MotorOnly(hardwareMap.get("leftForwardMotor") as DcMotorEx)
        val leftBackMotor = MotorOnly(hardwareMap.get("leftBackMotor") as DcMotorEx)
        val rightForwardMotor = MotorOnly(hardwareMap.get("rightForwardMotor") as DcMotorEx)
        val rightBackMotor = MotorOnly(hardwareMap.get("rightBackMotor") as DcMotorEx)

        leftForwardMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBackMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightForwardMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBackMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        leftBackMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()
        resetRuntime()

        leftForwardMotor.power = 0.3
        leftBackMotor.power = 0.3
        rightForwardMotor.power = 0.3
        rightBackMotor.power = 0.3

        while (opModeIsActive()){

        }
    }
}