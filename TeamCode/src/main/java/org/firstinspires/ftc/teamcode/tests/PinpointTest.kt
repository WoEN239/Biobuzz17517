package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(group = "tests")
class PinpointTest: LinearOpMode() {
    override fun runOpMode() {
        val pinpoint = hardwareMap.get("odometry") as GoBildaPinpointDriver

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)

        pinpoint.resetPosAndIMU()

        val multiplyTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()

        while (opModeIsActive()) {
            pinpoint.update()

            multiplyTelemetry.addData("x", pinpoint.encoderX)
            multiplyTelemetry.addData("y", pinpoint.encoderY)
            multiplyTelemetry.addData("hz", pinpoint.frequency)

            multiplyTelemetry.update()
        }
    }
}