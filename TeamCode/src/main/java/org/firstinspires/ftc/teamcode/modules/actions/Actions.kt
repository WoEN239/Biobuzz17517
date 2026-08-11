package org.firstinspires.ftc.teamcode.modules.actions

import com.qualcomm.robotcore.util.ElapsedTime

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