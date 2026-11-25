package org.firstinspires.ftc.teamcode.trajectoryes

import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.modules.actions.ActionLink
import org.firstinspires.ftc.teamcode.modules.actions.ActionsBuilder
import org.firstinspires.ftc.teamcode.modules.actions.DriveAction
import org.firstinspires.ftc.teamcode.modules.actions.FlowerDownAction
import org.firstinspires.ftc.teamcode.modules.actions.FlowerShootAction
import org.firstinspires.ftc.teamcode.modules.actions.FlowerUpAction
import org.firstinspires.ftc.teamcode.modules.actions.ShootAction
import org.firstinspires.ftc.teamcode.modules.actions.StartEatAction
import org.firstinspires.ftc.teamcode.modules.actions.StopEatAction
import org.firstinspires.ftc.teamcode.modules.actions.WaitAction
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveSegment
import org.firstinspires.ftc.teamcode.modules.driveTrain.MoveSegment
import org.firstinspires.ftc.teamcode.utils.units.Ang
import org.firstinspires.ftc.teamcode.utils.units.Pos
import org.firstinspires.ftc.teamcode.utils.units.Vec
import kotlin.math.PI

fun rightTrajectory(collector: Collector): ActionLink? {
    val bus = collector.eventBus

    val builder = ActionsBuilder()

    builder.next(WaitAction(3.3))
        .next(ShootAction(bus))
        .next(DriveAction(bus, DriveSegment(Pos(Vec(1.466, -1.126), Ang.ofDeg(-47.0)))))
        .next(StartEatAction(bus))
        .next(DriveAction(bus, DriveSegment(Pos(Vec(1.546, -1.757), Ang.ofDeg(0.0)), linearVelConstrain = 1.0, angularVelConstrain = 1.0)))
        .next(WaitAction(1.0))
        .next(StopEatAction(bus))
        .next(DriveAction(bus, DriveSegment(Pos(Vec(-1.198, -0.7), Ang.ofDeg(-180.0)))))
        .next(DriveAction(bus, DriveSegment(Pos(Vec(-1.198, -0.606), Ang.ofDeg(-180.0)))))
        .next(WaitAction(1.5))
        .next(ShootAction(bus))
        .next(StartEatAction(bus))
        .next(FlowerDownAction(bus))
        .next(WaitAction(0.2))
        .next(DriveAction(bus, DriveSegment(Pos(Vec(-1.918, -0.606), Ang.ofDeg(-180.0)), positionWindow = 1.0)))
        .next(FlowerShootAction(bus))
//        .next(FlowerUpAction(bus))
//        .next(DriveAction(bus, MoveSegment(Vec(0.138, -1.523))))
//        .next(StartEatAction(bus))
//        .next(DriveAction(bus, MoveSegment(Vec(-0.191, -1.523))))
//        .next(WaitAction(0.5))
//        .next(StopEatAction(bus))
//        .next(DriveAction(bus, DriveSegment(Pos(1.536, -0.730, 0.0))))
//        .next(ShootAction(bus))
        .next(DriveAction(bus, DriveSegment(Pos(-1.264, -1.361, PI / 2.0))))
//        .next(FlowerUpAction(bus))

    //(0.138, -1.523), -179.685°

    //(-0.191, -1.523), -179.613°
    //(0.138, -1.021), -179.668°

    //(0.613, -1.398), -90.996°

    //(1.536, -0.730), 0.375°

    return builder.build()
}