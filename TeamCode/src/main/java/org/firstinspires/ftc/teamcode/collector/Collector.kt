package org.firstinspires.ftc.teamcode.collector

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
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

    val runMode: RunMode

    constructor(opMode: LinearOpMode, runMode: RunMode) {
        this.runMode = runMode
        this.opMode = opMode
        this.hardwareMap = opMode.hardwareMap

        val hubs = hardwareMap.getAll(LynxModule::class.java)

        for (i in hubs)
            i.bulkCachingMode = LynxModule.BulkCachingMode.AUTO

        stopEvent += {
            for (i in opMode.hardwareMap.servoController) i.pwmDisable()
        }
    }
}