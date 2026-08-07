package org.firstinspires.ftc.teamcode.modules.utils

import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.collector.Collector

class Battery {
    var currentVoltage = 1.0
        private set

    fun voltageToPower(voltage: Double) = voltage / currentVoltage

    constructor(collector: Collector) {
        val voltageSensor = collector.hardwareMap.get(VoltageSensor::class.java, "Control Hub")

        collector.updateEvent += {
            currentVoltage = voltageSensor.voltage
        }

        collector.initUpdateEvent += {
            currentVoltage = voltageSensor.voltage
        }
    }
}