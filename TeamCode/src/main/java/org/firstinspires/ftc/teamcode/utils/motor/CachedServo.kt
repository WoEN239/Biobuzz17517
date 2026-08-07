package org.firstinspires.ftc.teamcode.utils.motor

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController
import kotlin.math.abs

class CachedServo : Servo {
    private val _servo: Servo

    constructor(servo: Servo) {
        _servo = servo
    }

    override fun getController(): ServoController? = _servo.controller

    override fun getPortNumber() = _servo.portNumber

    override fun setDirection(direction: Servo.Direction?) {
        _servo.direction = direction
    }

    override fun getDirection(): Servo.Direction? = _servo.direction

    private var _oldPosition: Double? = null

    override fun setPosition(position: Double) {
        if (_oldPosition == null)
            _oldPosition = position
        else {
            if (abs(_oldPosition!! - position) < 0.0001)
                return

            _oldPosition = position
        }

        _servo.position = position
    }

    override fun getPosition() = if (_oldPosition == null) 0.0 else _oldPosition!!

    override fun scaleRange(min: Double, max: Double) {
        _servo.scaleRange(min, max)
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer? = _servo.manufacturer

    override fun getDeviceName(): String? = _servo.deviceName

    override fun getConnectionInfo(): String? = _servo.connectionInfo

    override fun getVersion() = _servo.version

    override fun resetDeviceConfigurationForOpMode() {
        _servo.resetDeviceConfigurationForOpMode()
    }

    override fun close() {
        _servo.close()
    }
}