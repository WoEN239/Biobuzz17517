package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.collector.GameColor
import org.firstinspires.ftc.teamcode.collector.Settings

@TeleOp
class Bootloader : LinearOpMode() {
    override fun runOpMode() {
        val pinpoint = hardwareMap.get("odometry") as GoBildaPinpointDriver
        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        var isOdometryReseted = false
        var selectedGameColor = Settings.color.ordinal
        val gameColors = GameColor.entries

        OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().activity)
            .startActiveOpMode()

        waitForStart()
        resetRuntime()

        while (opModeIsActive()) {
            if (gamepad1.circleWasPressed()) {
                pinpoint.resetPosAndIMU()
                isOdometryReseted = true
            }

            if (gamepad1.dpadUpWasPressed()) {
                selectedGameColor++
                selectedGameColor %= gameColors.size
            }

            if (gamepad1.dpadDownWasPressed()) {
                selectedGameColor--

                if (selectedGameColor < 0)
                    selectedGameColor = gameColors.lastIndex
            }

            telemetry.addLine("selected game color ${gameColors[selectedGameColor]}")

            if (isOdometryReseted)
                telemetry.addLine("odometry reseted")

            telemetry.update()
        }
    }
}