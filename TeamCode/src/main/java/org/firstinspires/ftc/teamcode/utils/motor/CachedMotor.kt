package org.firstinspires.ftc.teamcode.utils.motor

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs

class CachedMotor: DcMotorEx {
    private val _motor: DcMotorEx
    private var _oldPower = 0.0

    constructor(motor: DcMotorEx) {
        _motor = motor
    }

    override fun setMotorEnable() {
        _motor.setMotorEnable()
    }

    override fun setMotorDisable() {
        _motor.setMotorDisable()
    }

    override fun isMotorEnabled() = _motor.isMotorEnabled

    override fun setVelocity(angularRate: Double) {
        _motor.velocity = angularRate
    }

    override fun setVelocity(
        angularRate: Double,
        unit: AngleUnit?
    ) {
        _motor.setVelocity(angularRate, unit)
    }

    override fun getVelocity() = _motor.velocity

    override fun getVelocity(unit: AngleUnit?) = _motor.getVelocity(unit)

    @Deprecated("Deprecated in Java")
    override fun setPIDCoefficients(
        mode: DcMotor.RunMode?,
        pidCoefficients: PIDCoefficients?
    ) {
        _motor.setPIDCoefficients(mode, pidCoefficients)
    }

    override fun setPIDFCoefficients(
        mode: DcMotor.RunMode?,
        pidfCoefficients: PIDFCoefficients?
    ) {
        _motor.setPIDFCoefficients(mode, pidfCoefficients)
    }

    override fun setVelocityPIDFCoefficients(
        p: Double,
        i: Double,
        d: Double,
        f: Double
    ) {
        _motor.setVelocityPIDFCoefficients(p, i, d, f)
    }

    override fun setPositionPIDFCoefficients(p: Double) {
        _motor.setPositionPIDFCoefficients(p)
    }

    @Deprecated("Deprecated in Java")
    override fun getPIDCoefficients(mode: DcMotor.RunMode?): PIDCoefficients? = _motor.getPIDCoefficients(mode)

    override fun getPIDFCoefficients(mode: DcMotor.RunMode?): PIDFCoefficients? = _motor.getPIDFCoefficients(mode)

    override fun setTargetPositionTolerance(tolerance: Int) {
        _motor.targetPositionTolerance = tolerance
    }

    override fun getTargetPositionTolerance() = _motor.targetPositionTolerance

    override fun getCurrent(unit: CurrentUnit?) = _motor.getCurrent(unit)

    override fun getCurrentAlert(unit: CurrentUnit?) = _motor.getCurrentAlert(unit)

    override fun setCurrentAlert(
        current: Double,
        unit: CurrentUnit?
    ) {
        _motor.setCurrentAlert(current, unit)
    }

    override fun isOverCurrent() = _motor.isOverCurrent

    override fun getMotorType(): MotorConfigurationType? = _motor.motorType

    override fun setMotorType(motorType: MotorConfigurationType?) {
        _motor.motorType = motorType
    }

    override fun getController(): DcMotorController?  = _motor.controller

    override fun getPortNumber() = _motor.portNumber

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        _motor.zeroPowerBehavior = zeroPowerBehavior
    }

    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior?  = _motor.zeroPowerBehavior

    @Deprecated("Deprecated in Java")
    override fun setPowerFloat() {
        _motor.setPowerFloat()
    }

    override fun getPowerFloat() = _motor.powerFloat

    override fun setTargetPosition(position: Int) {
       _motor.targetPosition = position
    }

    override fun getTargetPosition() = _motor.targetPosition

    override fun isBusy() = _motor.isBusy

    override fun getCurrentPosition() = _motor.currentPosition

    override fun setMode(mode: DcMotor.RunMode?) {
        _motor.mode = mode
    }

    override fun getMode(): DcMotor.RunMode? = _motor.mode

    override fun setDirection(direction: DcMotorSimple.Direction?) {
        _motor.direction = direction
    }

    override fun getDirection(): DcMotorSimple.Direction? = _motor.direction

    override fun setPower(power: Double) {
        if(abs(_oldPower - power) < 0.0001)
            return

        _oldPower = power

        _motor.power = power
    }

    override fun getPower() = _oldPower

    override fun getManufacturer(): HardwareDevice.Manufacturer?  = _motor.manufacturer

    override fun getDeviceName(): String?  = _motor.deviceName

    override fun getConnectionInfo(): String? = _motor.connectionInfo

    override fun getVersion() = _motor.version

    override fun resetDeviceConfigurationForOpMode() {
        _motor.resetDeviceConfigurationForOpMode()
    }

    override fun close() {
        _motor.close()
    }
}