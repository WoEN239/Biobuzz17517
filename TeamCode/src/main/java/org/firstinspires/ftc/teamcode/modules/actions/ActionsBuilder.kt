package org.firstinspires.ftc.teamcode.modules.actions

class ActionsBuilder {
    private var _lastAction: IAction? = null
    private var _firstAction: IAction? = null

    fun next(action: IAction? = null): ActionsBuilder {
        if (_firstAction == null) {
            _firstAction = action
            _lastAction = action
        } else {
            if (_lastAction!!.nextAction == null) {
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
    ) = next(BranchAction(condition, trueActions, falseActions))

    fun paralelOr(vararg actions: IAction?) =
        next(ParallelActions(actions.toList().toTypedArray(), ParallelActions.ExitType.OR))

    fun paralelAnd(vararg actions: IAction?) =
        next(ParallelActions(actions.toList().toTypedArray(), ParallelActions.ExitType.AND))

    fun paralel(vararg actions: IAction?) = paralelAnd(*actions)

    fun solo(action: IAction) = next(SoloAction(action))

    fun run(action: () -> Unit) = next(object : IAction() {
        override fun start() = action()
    })

    fun build(): IAction? {
        return _firstAction
    }

    fun lastAction() = _lastAction
}