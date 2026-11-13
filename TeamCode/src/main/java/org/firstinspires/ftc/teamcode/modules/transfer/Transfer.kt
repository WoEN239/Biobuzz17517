package org.firstinspires.ftc.teamcode.modules.transfer

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.collector.RunMode
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
    var SHOOTING_POWER = 1.0

    @JvmField
    var SHOOT_TIMER = 0.1

    @JvmField
    var FLOWER_UP = 0.0

    @JvmField
    var FLOWER_DOWN = 0.5

    @JvmField
    var SHOOT_TIME = 1.5
}

class SwitchTransferStateEvent(val newState: TransferState)

fun attachTransfer(collector: Collector) {
    val flowerServo = collector.hardwareMap.get("flowerServo") as Servo

    val transferMotor = MotorOnly(collector.hardwareMap.get("transferMotor") as DcMotorEx)
    val brushMotor = MotorOnly(collector.hardwareMap.get("brushMotor") as DcMotorEx)

    transferMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    transferMotor.direction = DcMotorSimple.Direction.REVERSE

    brushMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    val turretDoorServo = CachedServo(collector.hardwareMap.get("turretDoorServo") as Servo)

    var state = TransferState.NONE

    var oldState = TransferState.NONE

    var brushState = true

    val stateTimer = ElapsedTime()

    fun switchState(newState: TransferState) {
        if (newState == state)
            return

        oldState = state

        state = newState

        stateTimer.reset()
    }

    collector.eventBus.sub(SwitchTransferStateEvent::class) {
        switchState(it.newState)

        brushState = it.newState == TransferState.SHUTTING || it.newState == TransferState.EAT
    }

    collector.startEvent += {
        flowerServo.position = TRANSFER_CONFIG.FLOWER_UP
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
            } else {
                brushState = gamepadData.left_trigger > 0.01

                if (gamepadData.right_trigger > 0.01)
                    switchState(TransferState.SHUTTING)
                else if (brushState)
                    switchState(TransferState.EAT)
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
                transferMotor.power =
                    if (stateTimer.seconds() > TRANSFER_CONFIG.SHOOT_TIMER) TRANSFER_CONFIG.SHOOTING_POWER else 0.0

                turretDoorServo.position = TRANSFER_CONFIG.DOOR_SERVO_OPEN
            }

            TransferState.NONE -> {
                transferMotor.power =
                    when (oldState) {
                        TransferState.SHUTTING if stateTimer.seconds() < 0.05 -> -1.0
                        TransferState.EAT if stateTimer.seconds() < 0.05 -> 1.0
                        else
                            -> 0.0
                    }

                turretDoorServo.position = TRANSFER_CONFIG.DOOR_SERVO_CLOSE
            }
        }

        brushMotor.power = if (brushState || collector.runMode == RunMode.AUTO) 1.0 else -TRANSFER_CONFIG.BRUSH_REVERSE_POWER
    }
}