package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.utils.Reg
import org.firstinspires.ftc.teamcode.utils.RegParams

@Config
internal object REGULATOR_TEST_CONFIG {
    @JvmField
    var REGULATOR = RegParams(limitU = 1.0)

    @JvmField
    var TARGET = 500.0
}

@TeleOp(group = "tests")
class RegulatorTest: LinearOpMode() {
    override fun runOpMode() {
        val motor = hardwareMap.get("shooterMotor") as DcMotorEx

        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        val regulator = Reg(REGULATOR_TEST_CONFIG.REGULATOR)

        val multTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()

        regulator.start()

        while (opModeIsActive()){
            val err = TURRET_ROTATE_TEST_CONFIG.TARGET - motor.velocity

            motor.power = regulator.update(err, TURRET_ROTATE_TEST_CONFIG.TARGET, 1.0)

            multTelemetry.addData("target", TURRET_ROTATE_TEST_CONFIG.TARGET)
            multTelemetry.addData("current", motor.velocity)
            multTelemetry.update()
        }
    }
}