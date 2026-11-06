package org.firstinspires.ftc.teamcode.modules.transfer

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.modules.utils.GamepadListener
import org.firstinspires.ftc.teamcode.modules.utils.IGamepadListener
import org.firstinspires.ftc.teamcode.utils.motor.CachedServo
import org.firstinspires.ftc.teamcode.utils.motor.MotorOnly

enum class TransferState {
    EAT,
    SHUTTING,
    REVERSE,
    NONE
}

@Config
internal object TRANSFER_CONFIG {
    @JvmField
    var DOOR_SERVO_OPEN = 0.38

    @JvmField
    var DOOR_SERVO_CLOSE = 0.54

    @JvmField
    var BRUSH_REVERSE_POWER = 0.45

    @JvmField
    var SHOOTING_POWER = 0.85
}

fun attachTransfer(collector: Collector) {
    val transferMotor = MotorOnly(collector.hardwareMap.get("transferMotor") as DcMotorEx)
    val brushMotor = MotorOnly(collector.hardwareMap.get("brushMotor") as DcMotorEx)

    transferMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    transferMotor.direction = DcMotorSimple.Direction.REVERSE

    brushMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    val turretDoorServo = CachedServo(collector.hardwareMap.get("turretDoorServo") as Servo)

    var state = TransferState.NONE

    var brushState = true

    fun switchState(newState: TransferState) {
        if (newState == state)
            return

        state = newState
    }

    collector.gamepad.addGamepad1Listener(object : IGamepadListener {
        override fun update(gamepadData: Gamepad) {
//            switchState(
//                if (gamepadData.right_bumper)
//                    TransferState.SHUTTING
//                else if (gamepadData.left_bumper)
//                    TransferState.EAT
//                else if (gamepadData.left_trigger > 0.01)
//                    TransferState.REVERSE
//                else
//                    TransferState.NONE
//            )

            if (gamepadData.left_bumper) {
                switchState(TransferState.REVERSE)
                brushState = false
            }
            else {
                brushState = gamepadData.left_trigger > 0.01

                if(gamepadData.right_trigger > 0.01)
                    switchState( TransferState.SHUTTING)
                else if(brushState)
                    switchState( TransferState.EAT)
                else
                    switchState(TransferState.NONE)
            }
        }
    })

    collector.updateEvent += {
        when (state) {
            TransferState.REVERSE -> {
                transferMotor.power = -1.0
                turretDoorServo.position = TRANSFER_CONFIG.DOOR_SERVO_CLOSE
            }

            TransferState.EAT -> {
                transferMotor.power = TRANSFER_CONFIG.SHOOTING_POWER
                turretDoorServo.position = TRANSFER_CONFIG.DOOR_SERVO_CLOSE
            }

            TransferState.SHUTTING -> {
                transferMotor.power = TRANSFER_CONFIG.SHOOTING_POWER

                turretDoorServo.position = TRANSFER_CONFIG.DOOR_SERVO_OPEN
            }

            TransferState.NONE -> {
                transferMotor.power = 0.0
                turretDoorServo.position = TRANSFER_CONFIG.DOOR_SERVO_CLOSE
            }
        }

        brushMotor.power = if(brushState) 1.0 else -TRANSFER_CONFIG.BRUSH_REVERSE_POWER
    }
}