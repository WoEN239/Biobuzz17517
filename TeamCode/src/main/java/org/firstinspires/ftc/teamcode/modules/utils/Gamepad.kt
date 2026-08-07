package org.firstinspires.ftc.teamcode.modules.utils

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.collector.RunMode
import java.util.LinkedList

interface IGamepadListener {
    fun update(gamepadData: Gamepad)
}

class GamepadListener(
    val activateState: Boolean, val buttonSuppler: (Gamepad) -> Boolean,
    val onTriggered: () -> Unit
) : IGamepadListener {
    override fun update(gamepadData: Gamepad) {
        if (buttonSuppler(gamepadData) == activateState)
            onTriggered()
    }
}

class AnalogGamepadListener(
    val inputSuppler: (Gamepad) -> Double,
    val onTriggered: (Double) -> Unit
) : IGamepadListener {
    override fun update(gamepadData: Gamepad) {
        val data = inputSuppler(gamepadData)

        onTriggered(data)
    }
}

class Gamepad {
    private val _gamepad1Listeners = LinkedList<IGamepadListener>()
    private val _gamepad2Listeners = LinkedList<IGamepadListener>()

    constructor(collector: Collector){
        if(collector.runMode == RunMode.AUTO)
            return

        collector.updateEvent += {
            for(i in _gamepad1Listeners)
                i.update(collector.opMode.gamepad1)

            for(i in _gamepad2Listeners)
                i.update(collector.opMode.gamepad2)
        }
    }

    fun addGamepad1Listener(listener: IGamepadListener) {
        _gamepad1Listeners.add(listener)
    }

    fun addGamepad2Listener(listener: IGamepadListener) {
        _gamepad2Listeners.add(listener)
    }
}