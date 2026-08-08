package org.firstinspires.ftc.teamcode.modules.actions

class ActionsBuilder {
    private var _lastAction: IAction? = null
    private var _firstAction: IAction? = null

    fun next(action: IAction): ActionsBuilder {
        if (_firstAction == null) {
            _firstAction = action
            _lastAction = action
        } else {
            _lastAction!!.nextAction = action
            _lastAction = action
        }

        return this
    }

    fun branch(
        condition: () -> Boolean,
        trueActions: IAction,
        falseActions: IAction? = null
    ): ActionsBuilder {
        val branchAction = BranchAction(condition, trueActions, falseActions)

        if (_firstAction == null) {
            _firstAction = branchAction
            _lastAction = branchAction
        } else {
            _lastAction!!.nextAction = branchAction
            _lastAction = branchAction
        }

        return this
    }

    fun build(): IAction? {
        var currentAction = _firstAction

        while (currentAction != null) {
            currentAction.build()
            currentAction = currentAction.nextAction
        }

        return _firstAction
    }
}