package org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.collector.RunMode
import kotlin.math.absoluteValue

@TeleOp
class TeleOp : LinearOpMode() {
    override fun runOpMode() {
        val collector = Collector(this, RunMode.AUTO)

        while (!isStarted()) {
            collector.initUpdateEvent()

            if (gamepad1.left_stick_x.absoluteValue > 0.01 || gamepad1.left_stick_y.absoluteValue > 0.01 ||
                gamepad1.right_stick_x.absoluteValue > 0.01 || gamepad1.right_stick_y.absoluteValue > 0.01 ||
                gamepad1.left_trigger > 0.01 || gamepad1.right_trigger > 0.01 || gamepad1.left_bumper ||
                gamepad1.right_bumper || gamepad1.ps || gamepad1.touchpad || gamepad1.dpad_up ||
                gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right ||
                gamepad1.circle || gamepad1.square || gamepad1.triangle || gamepad1.cross
            )
                OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().activity)
                    .startActiveOpMode()
        }

        resetRuntime()

        collector.startEvent()

        while (opModeIsActive()) {
            collector.updateEvent()

            telemetry.update()
        }

        collector.stopEvent()
    }
}