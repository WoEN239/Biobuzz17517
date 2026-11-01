package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime

@TeleOp(group = "tests")
class DoubleLimelightTest: LinearOpMode() {
    override fun runOpMode() {
        val limelight1 = hardwareMap.get("Lime light 1") as Limelight3A
        val limelight2 = hardwareMap.get("Lime light 2") as Limelight3A

        val multableTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val deltaTime = ElapsedTime()

        waitForStart()

        while (opModeIsActive()){
            val updates = 1.0 / deltaTime.seconds()
            deltaTime.reset()

            multableTelemetry.addData("upd", updates)

//            multableTelemetry.addLine("l1 ${limelight1.status}")
//            multableTelemetry.addLine("l2 ${limelight2.status}")

            limelight1.status
            limelight2.status

            multableTelemetry.update()
        }
    }
}