package org.firstinspires.ftc.teamcode.modules.actions

import com.qualcomm.robotcore.util.ElapsedTime

abstract class IAction() {
    open fun start() {}
    open fun update() {}
    open fun stop(force: Boolean) {}

    open fun build() {}

    open fun isEnd(): Boolean = true

    var nextAction: IAction? = null
}

class WaitAction(private val _time: Double) : IAction() {
    val timer = ElapsedTime()

    override fun start() {
        timer.reset()
    }

    override fun isEnd() = timer.seconds() > _time
}

class ParallelActions(
    private val _actions: Array<IAction?>,
    private val _exitType: ExitType = ExitType.AND
) : IAction() {
    private var _actionsClone: Array<IAction?>? = null

    enum class ExitType {
        AND, OR
    }

    override fun update() {
        for (i in _actionsClone!!.indices) {
            if (_actionsClone!![i] != null) {
                _actionsClone!![i]!!.update()

                if (_actionsClone!![i]!!.isEnd()) {
                    _actionsClone!![i]!!.stop(false)
                    _actionsClone!![i] = _actionsClone!![i]!!.nextAction
                    _actionsClone!![i]?.start()
                }
            }
        }
    }

    override fun isEnd(): Boolean {
        if (_exitType == ExitType.AND) {
            for (i in _actionsClone!!) {
                if (i != null)
                    return false
            }

            return true
        } else {
            for (i in _actionsClone!!) {
                if (i == null) {
                    for (j in _actionsClone!!)
                        j?.stop(true)

                    return true
                }
            }

            return false
        }
    }

    override fun start() {
        _actionsClone = _actions.clone()

        for (i in _actionsClone!!)
            i?.start()
    }

    override fun stop(force: Boolean) {
        if (force) {
            for (j in _actionsClone!!)
                j?.stop(true)
        }
    }

    override fun build() {
        for (i in _actions) {
            var currentAction = i

            while (currentAction != null) {
                currentAction.build()
                currentAction = currentAction.nextAction
            }
        }
    }
}

class BranchAction(
    private val _condition: () -> Boolean,
    private val _trueActions: IAction,
    private val _falseActions: IAction? = null
) : IAction() {
    private var _currentAction: IAction? = null

    override fun start() {
        _currentAction = if (_condition()) _trueActions else _falseActions

        _currentAction?.start()
    }

    override fun update() {
        if (_currentAction != null) {
            _currentAction!!.update()

            if (_currentAction!!.isEnd()) {
                _currentAction!!.stop(false)
                _currentAction = _currentAction!!.nextAction
                _currentAction?.start()
            }
        }
    }

    override fun isEnd() = _currentAction == null

    override fun build() {
        var currentAction: IAction? = _trueActions

        while (currentAction != null) {
            currentAction.build()
            currentAction = currentAction.nextAction
        }

        currentAction = _falseActions

        while (currentAction != null) {
            currentAction.build()
            currentAction = currentAction.nextAction
        }
    }
}

class SoloAction(val action: IAction) : IAction() {
    override fun start() {
        action.start()
    }

    override fun stop(force: Boolean) {
        action.stop(force)
    }

    override fun update() {
        action.update()
    }

    override fun isEnd() = action.isEnd()

    override fun build() = action.build()
}