package org.firstinspires.ftc.teamcode.trajectoryes

import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.modules.actions.ActionLink
import org.firstinspires.ftc.teamcode.modules.actions.ActionsBuilder
import org.firstinspires.ftc.teamcode.modules.actions.DriveAction
import org.firstinspires.ftc.teamcode.modules.actions.IAction
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveSegment
import org.firstinspires.ftc.teamcode.modules.driveTrain.MoveSegment
import org.firstinspires.ftc.teamcode.utils.events.EventBus
import org.firstinspires.ftc.teamcode.utils.units.Ang
import org.firstinspires.ftc.teamcode.utils.units.Pos
import org.firstinspires.ftc.teamcode.utils.units.Vec

fun testTrajectory(collector: Collector): ActionLink? {
    val bus = collector.eventBus

    val builder = ActionsBuilder()

    builder.next(DriveAction(bus, DriveSegment(Pos(Vec(1.0, 1.0), Ang.ofDeg(0.0)))))
    builder.next(DriveAction(bus, DriveSegment(Pos(Vec(1.0, -1.0), Ang.ofDeg(90.0)))))
    builder.next(DriveAction(bus, DriveSegment(Pos(Vec(-1.0, -1.0), Ang.ofDeg(180.0)))))
    builder.next(DriveAction(bus, DriveSegment(Pos(Vec(-1.0, 1.0), Ang.ofDeg(270.0)))))
    builder.next(builder.first())

    return builder.build()
}