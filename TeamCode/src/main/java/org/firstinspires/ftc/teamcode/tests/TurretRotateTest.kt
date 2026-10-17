package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.drivers.SOFT_SERVO_EX_CONFIG
import org.firstinspires.ftc.teamcode.utils.drivers.SoftServoEx

@Config
internal object TURRET_ROTATE_TEST_CONFIG {
    @JvmField
    var TARGET = 0.1
}

@TeleOp(group = "tests")
class TurretRotateTest: LinearOpMode() {
    override fun runOpMode() {
        val serv = SoftServoEx("serv", hardwareMap, TURRET_ROTATE_TEST_CONFIG.TARGET, SOFT_SERVO_EX_CONFIG.SERVO_A,
            SOFT_SERVO_EX_CONFIG.SERVO_V_MAX)

        val multiTelemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        waitForStart()
        resetRuntime()

        serv.start()

        while(opModeIsActive()){
            serv.update()

            serv.targetPosition = TURRET_ROTATE_TEST_CONFIG.TARGET

            multiTelemetry.addData("current", serv.currentPosition)
            multiTelemetry.addData("target", serv.targetPosition)

            multiTelemetry.update()
        }
    }
}