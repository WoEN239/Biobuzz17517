package org.firstinspires.ftc.teamcode.modules.actions

class ActionsBuilder {
    private var _lastAction: IAction? = null
    private var _firstAction: IAction? = null

    fun next(action: IAction?): ActionsBuilder {
        if (_firstAction == null) {
            _firstAction = action
            _lastAction = action
        } else {
            if(_lastAction!!.nextAction == null) {
                _lastAction!!.nextAction = action
                _lastAction = action
            }
        }

        return this
    }

    fun branch(
        condition: () -> Boolean,
        trueActions: IAction,
        falseActions: IAction? = null
    ): ActionsBuilder {
        val branchAction = BranchAction(condition, trueActions, falseActions)

        next(branchAction)

        return this
    }

    fun paralelOr(vararg actions: IAction): ActionsBuilder {
        val paralelAction = ParallelActions(actions.toList().toTypedArray(), ParallelActions.ExitType.OR)

        next(paralelAction)

        return this
    }

    fun paralelAnd(vararg actions: IAction): ActionsBuilder {
        val paralelAction = ParallelActions(actions.toList().toTypedArray(), ParallelActions.ExitType.AND)

        next(paralelAction)

        return this
    }

    fun paralel(vararg actions: IAction): ActionsBuilder {
        paralelAnd(*actions)

        return this
    }

    fun build(): IAction? {
        return _firstAction
    }

    fun lastAction() = _lastAction
}