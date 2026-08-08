package org.firstinspires.ftc.teamcode.modules.actions

import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.trajectoryes.testTrajectory

fun attachActionRunner(collector: Collector) {
    var currentAction: IAction? = testTrajectory(collector.eventBus)

    collector.startEvent += {
        currentAction?.start()
    }

    collector.updateEvent += {
        if (currentAction != null) {
            currentAction!!.update()

            if (currentAction!!.isEnd()) {
                currentAction!!.stop(false)
                currentAction = currentAction!!.nextAction
                currentAction?.start()
            }
        }
    }

    collector.stopEvent += {
        currentAction?.stop(true)
    }
}