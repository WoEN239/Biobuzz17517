package org.firstinspires.ftc.teamcode.modules.actions

class ActionLink(val action: IAction) {
    var nextAction: ActionLink? = null
}

class ActionsBuilder {
    private var _lastAction: ActionLink? = null
    private var _firstAction: ActionLink? = null

    fun next(action: IAction) = next(ActionLink(action))

    fun next(link: ActionLink?): ActionsBuilder {
        if (_firstAction == null) {
            _firstAction = link
            _lastAction = link
        } else {
            if (_lastAction!!.nextAction == null) {
                _lastAction!!.nextAction = link
                _lastAction = link
            }
        }

        return this
    }

    fun branch(
        condition: () -> Boolean,
        trueActions: IAction,
        falseActions: IAction? = null
    ) = branch(
        condition,
        ActionLink(trueActions),
        if (falseActions == null) null else ActionLink(falseActions)
    )

    fun branch(
        condition: () -> Boolean,
        trueActions: ActionLink,
        falseActions: ActionLink? = null
    ) = next(BranchAction(condition, trueActions, falseActions))

    fun paralelOr(vararg actions: ActionLink?) =
        next(ParallelActions(actions.toList(), ParallelActions.ExitType.OR))

    fun paralelOr(vararg actions: IAction?) =
        next(
            ParallelActions(
                actions.map { if (it == null) null else ActionLink(it) },
                ParallelActions.ExitType.OR
            )
        )

    fun paralelAnd(vararg actions: ActionLink?) =
        next(ParallelActions(actions.toList(), ParallelActions.ExitType.AND))

    fun paralelAnd(vararg actions: IAction?) =
        next(
            ParallelActions(
                actions.map { if (it == null) null else ActionLink(it) },
                ParallelActions.ExitType.AND
            )
        )

    fun paralel(vararg actions: ActionLink?) = paralelAnd(*actions)
    fun paralel(vararg actions: IAction?) = paralelAnd(*actions)

    fun run(action: () -> Unit) = next(object : IAction {
        override fun start() = action()
    })

    fun build(): ActionLink? {
        return _firstAction
    }

    fun last() = _lastAction
}