package org.firstinspires.ftc.teamcode.modules.actions

import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.trajectoryes.rightTrajectory

fun attachActionRunner(collector: Collector) {
    var currentAction: ActionLink? = rightTrajectory(collector)

    collector.startEvent += {
        currentAction?.action?.start()
    }

    collector.updateEvent += {
        if (currentAction != null) {
            currentAction!!.action.update()

            if (currentAction!!.action.isEnd()) {
                currentAction!!.action.stop(false)
                currentAction = currentAction!!.nextAction
                currentAction?.action?.start()
            }
        }
    }

    collector.stopEvent += {
        currentAction?.action?.stop(true)
    }
}