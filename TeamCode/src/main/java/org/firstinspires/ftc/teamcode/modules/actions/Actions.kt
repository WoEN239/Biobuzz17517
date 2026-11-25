package org.firstinspires.ftc.teamcode.modules.actions

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveSegment
import org.firstinspires.ftc.teamcode.modules.driveTrain.GetRunnerAtTargetAngleEvent
import org.firstinspires.ftc.teamcode.modules.driveTrain.GetRunnerAtTargetPositionEvent
import org.firstinspires.ftc.teamcode.modules.driveTrain.ITrajectorySegment
import org.firstinspires.ftc.teamcode.modules.driveTrain.MoveSegment
import org.firstinspires.ftc.teamcode.modules.driveTrain.RunSegmentEvent
import org.firstinspires.ftc.teamcode.modules.driveTrain.TurnSegment
import org.firstinspires.ftc.teamcode.modules.transfer.SwitchFlowerStateEvent
import org.firstinspires.ftc.teamcode.modules.transfer.SwitchTransferStateEvent
import org.firstinspires.ftc.teamcode.modules.transfer.TRANSFER_CONFIG
import org.firstinspires.ftc.teamcode.modules.transfer.TransferState
import org.firstinspires.ftc.teamcode.utils.events.EventBus
import java.util.LinkedList

@Config
internal object ACTIONS_CONFIG {
    @JvmField
    var FLOWER_SHOOT_TIMER = 2.0

    @JvmField
    var FLOWER_SHOOT_REVERS_TIMER = 0.5

    @JvmField
    var FLOWER_SECONDARY_SHOOT_TIMER = 1.0
}

interface IAction {
    fun start() {}
    fun update() {}
    fun stop(force: Boolean) {}

    fun isEnd(): Boolean = true
}

class WaitAction(private val _time: Double) : IAction {
    val timer = ElapsedTime()

    override fun start() {
        timer.reset()
    }

    override fun isEnd() = timer.seconds() > _time
}

class ParallelActions(
    private val _actions: List<ActionLink?>,
    private val _exitType: ExitType = ExitType.AND
) : IAction {
    private var _actionsClone = mutableListOf<ActionLink?>()

    enum class ExitType {
        AND, OR
    }

    override fun update() {
        for (i in _actionsClone.indices) {
            if (_actionsClone[i] != null) {
                val action = _actionsClone[i]!!.action
                action.update()

                if (action.isEnd()) {
                    action.stop(false)
                    _actionsClone[i] = _actionsClone[i]!!.nextAction
                    _actionsClone[i]?.action?.start()
                }
            }
        }
    }

    override fun isEnd(): Boolean {
        if (_exitType == ExitType.AND) {
            for (i in _actionsClone) {
                if (i != null)
                    return false
            }

            return true
        } else {
            for (i in _actionsClone) {
                if (i == null) {
                    for (j in _actionsClone)
                        j?.action?.stop(true)

                    return true
                }
            }

            return false
        }
    }

    override fun start() {
        _actionsClone = _actions.toMutableList()

        for (i in _actionsClone)
            i?.action?.start()
    }

    override fun stop(force: Boolean) {
        if (force) {
            for (j in _actionsClone)
                j?.action?.stop(true)
        }
    }
}

class BranchAction(
    private val _condition: () -> Boolean,
    private val _trueActions: ActionLink,
    private val _falseActions: ActionLink? = null
) : IAction {
    private var _currentAction: ActionLink? = null

    override fun start() {
        _currentAction = if (_condition()) _trueActions else _falseActions

        _currentAction?.action?.start()
    }

    override fun update() {
        if (_currentAction != null) {
            val action = _currentAction!!.action
            action.update()

            if (action.isEnd()) {
                action.stop(false)
                _currentAction = _currentAction!!.nextAction
                _currentAction?.action?.start()
            }
        }
    }

    override fun isEnd() = _currentAction == null
}

class DriveAction(val eventBus: EventBus, vararg segments: ITrajectorySegment) : IAction {
    val segmentsList = segments.toList()
    var segmentsCopy: LinkedList<ITrajectorySegment>? = null

    var atTargetAngleConsumer = { true }
    var atTargetPositionConsumer = { true }

    override fun start() {
        segmentsCopy = LinkedList(segmentsList)

        atTargetPositionConsumer = eventBus(GetRunnerAtTargetPositionEvent()).atTarget
        atTargetAngleConsumer = eventBus(GetRunnerAtTargetAngleEvent()).atTarget

        eventBus(RunSegmentEvent(segmentsCopy!!.first()))
    }

    override fun update() {
        if (segmentsCopy!!.isNotEmpty()) {
            val first = segmentsCopy!!.first()

            if ((first is MoveSegment && atTargetPositionConsumer()) ||
                (first is TurnSegment && atTargetAngleConsumer()) ||
                (first is DriveSegment && atTargetPositionConsumer() && atTargetAngleConsumer())
            ) {
                segmentsCopy?.removeFirst()

                if (segmentsCopy!!.isNotEmpty())
                    eventBus(RunSegmentEvent(segmentsCopy!!.first()))
            }
        }
    }

    override fun isEnd() = segmentsCopy!!.isEmpty()
}

class ShootAction(val eventBus: EventBus) : IAction {
    val timer = ElapsedTime()

    override fun start() {
        timer.reset()

        eventBus.invoke(SwitchTransferStateEvent(TransferState.SHUTTING))
    }

    override fun stop(force: Boolean) {
        eventBus.invoke(SwitchTransferStateEvent(TransferState.NONE))
    }

    override fun isEnd() = timer.seconds() > TRANSFER_CONFIG.SHOOT_TIME
}

class FlowerShootAction(val eventBus: EventBus) : IAction {
    var state = 0
    val timer = ElapsedTime()

    override fun start() {
        timer.reset()

        state = 0

        eventBus.invoke(SwitchTransferStateEvent(TransferState.SHUTTING))
    }

    override fun update() {
        if (state == 0 && timer.seconds() > ACTIONS_CONFIG.FLOWER_SHOOT_TIMER) {
            state = 1

            eventBus.invoke(SwitchTransferStateEvent(TransferState.REVERSE))
        }

        if (state == 1 && timer.seconds() > ACTIONS_CONFIG.FLOWER_SHOOT_TIMER + ACTIONS_CONFIG.FLOWER_SHOOT_REVERS_TIMER) {
            eventBus.invoke(SwitchTransferStateEvent(TransferState.SHUTTING))
            state = 2
        }
    }

    override fun stop(force: Boolean) {
        eventBus.invoke(SwitchTransferStateEvent(TransferState.NONE))
    }

    override fun isEnd() =
        state == 2 && timer.seconds() > ACTIONS_CONFIG.FLOWER_SHOOT_TIMER + ACTIONS_CONFIG.FLOWER_SHOOT_REVERS_TIMER + ACTIONS_CONFIG.FLOWER_SECONDARY_SHOOT_TIMER
}

class StartEatAction(val eventBus: EventBus) : IAction {
    override fun start() {
        eventBus.invoke(SwitchTransferStateEvent(TransferState.EAT))
    }
}

class StopEatAction(val eventBus: EventBus) : IAction {
    override fun start() {
        eventBus.invoke(SwitchTransferStateEvent(TransferState.NONE))
    }
}

class FlowerDownAction(val eventBus: EventBus) : IAction {
    override fun start() {
        eventBus.invoke(SwitchFlowerStateEvent(true))
    }
}


class FlowerUpAction(val eventBus: EventBus) : IAction {
    override fun start() {
        eventBus.invoke(SwitchFlowerStateEvent(false))
    }
}