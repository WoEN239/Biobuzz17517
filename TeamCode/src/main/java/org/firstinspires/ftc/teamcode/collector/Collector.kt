package org.firstinspires.ftc.teamcode.collector

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.modules.utils.Battery
import org.firstinspires.ftc.teamcode.modules.utils.Gamepad
import org.firstinspires.ftc.teamcode.modules.utils.Telemetry
import org.firstinspires.ftc.teamcode.utils.events.EventBus
import org.firstinspires.ftc.teamcode.utils.events.SimpleEmptyEvent

enum class RunMode {
    AUTO,
    MANUAL
}

class Collector {
    val startEvent = SimpleEmptyEvent()
    val initUpdateEvent = SimpleEmptyEvent()
    val updateEvent = SimpleEmptyEvent()
    val stopEvent = SimpleEmptyEvent()

    val opMode: LinearOpMode

    val eventBus = EventBus()
    val hardwareMap: HardwareMap
    val telemetry: Telemetry
    val battery: Battery
    val gamepad: Gamepad

    val runMode: RunMode

    constructor(opMode: LinearOpMode, runMode: RunMode) {
        this.runMode = runMode
        this.opMode = opMode
        hardwareMap = opMode.hardwareMap
        telemetry = Telemetry(this)
        battery = Battery(this)
        gamepad = Gamepad(this)

        val hubs = hardwareMap.getAll(LynxModule::class.java)

        for (i in hubs)
            i.bulkCachingMode = LynxModule.BulkCachingMode.AUTO

        stopEvent += {
            for (i in opMode.hardwareMap.servoController) i.pwmDisable()
        }

        val deltaTime = ElapsedTime()

        startEvent += {
            deltaTime.reset()
        }

        updateEvent += {
            telemetry.addData("ups", 1.0 / deltaTime.seconds())
        }
    }
}