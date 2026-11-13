package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.collector.Settings
import org.firstinspires.ftc.teamcode.collector.StartOrientation
import org.firstinspires.ftc.teamcode.modules.driveTrain.ODOMETRY_CONFIG

@TeleOp
class Bootloader : LinearOpMode() {
    override fun runOpMode() {
        val pinpoint = hardwareMap.get("odometry") as GoBildaPinpointDriver

        pinpoint.setOffsets(
            ODOMETRY_CONFIG.X_POD_POSITION,
            ODOMETRY_CONFIG.Y_POD_POSITION,
            DistanceUnit.METER
        )

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        )

        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        var isOdometryReseted = false

        var selectedGameOrientation = Settings.orientation.ordinal
        val orientations = StartOrientation.entries

        OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().activity)
            .startActiveOpMode()

        waitForStart()
        resetRuntime()

        while (opModeIsActive()) {
            if (gamepad1.circleWasPressed()) {
                pinpoint.resetPosAndIMU()
//                pinpoint.setHeading(0.0, AngleUnit.RADIANS)
//                pinpoint.setPosX(0.172, DistanceUnit.METER)
//                pinpoint.setPosY(-1.033, DistanceUnit.METER)

                isOdometryReseted = true
            }

            if (gamepad1.dpadUpWasPressed()) {
                selectedGameOrientation++
                selectedGameOrientation %= orientations.size
            }

            if (gamepad1.dpadDownWasPressed()) {
                selectedGameOrientation--

                if (selectedGameOrientation < 0)
                    selectedGameOrientation = orientations.lastIndex
            }

            telemetry.addLine("selected game orientation ${orientations[selectedGameOrientation]}")

            if (isOdometryReseted)
                telemetry.addLine("odometry reseted")

            telemetry.update()
        }

        Settings.orientation = orientations[selectedGameOrientation]
    }
}