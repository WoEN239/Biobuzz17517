package org.firstinspires.ftc.teamcode.trajectoryes

import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.modules.actions.ActionLink
import org.firstinspires.ftc.teamcode.modules.actions.ActionsBuilder
import org.firstinspires.ftc.teamcode.modules.actions.DriveAction
import org.firstinspires.ftc.teamcode.modules.actions.ShootAction
import org.firstinspires.ftc.teamcode.modules.actions.StartEatAction
import org.firstinspires.ftc.teamcode.modules.actions.StopEatAction
import org.firstinspires.ftc.teamcode.modules.actions.WaitAction
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveSegment
import org.firstinspires.ftc.teamcode.modules.driveTrain.MoveSegment
import org.firstinspires.ftc.teamcode.utils.units.Ang
import org.firstinspires.ftc.teamcode.utils.units.Pos
import org.firstinspires.ftc.teamcode.utils.units.Vec

fun rightTrajectory(collector: Collector): ActionLink? {
    val bus = collector.eventBus

    val builder = ActionsBuilder()

    builder.next(WaitAction(3.3))
        .next(ShootAction(bus))
        .next(DriveAction(bus, DriveSegment(Pos(Vec(1.466, -1.126), Ang.ofDeg(-47.0)))))
        .next(StartEatAction(bus))
        .next(DriveAction(bus, DriveSegment(Pos(Vec(1.546, -1.697), Ang.ofDeg(0.0)), linearVelConstrain = 1.0, angularVelConstrain = 1.0)))
        .next(WaitAction(1.0))
        .next(StopEatAction(bus))
        .next(DriveAction(bus, DriveSegment(Pos(Vec(-1.398, -0.506), Ang.ofDeg(-180.0)))))
        .next(WaitAction(1.0))
        .next(ShootAction(bus))
        .next(DriveAction(bus, MoveSegment(Vec(-0.895, -1.529))))

    return builder.build()
}