package org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.collector.RunMode

@Autonomous
class Auto : LinearOpMode() {
    override fun runOpMode() {
        val collector = Collector(this, RunMode.AUTO)

        do {
            collector.initUpdateEvent()
        } while (!isStarted())

        waitForStart()
        resetRuntime()

        collector.startEvent()

        while (opModeIsActive()) {
            collector.updateEvent()

            telemetry.update()
        }

        collector.stopEvent()

        OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().activity)
            .initOpMode(TeleOp::class.simpleName)
    }
}