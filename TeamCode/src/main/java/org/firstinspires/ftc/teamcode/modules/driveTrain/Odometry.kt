package org.firstinspires.ftc.teamcode.modules.driveTrain

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.IMU
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.utils.ExponentialFilter
import org.firstinspires.ftc.teamcode.utils.units.Ang
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Pos
import org.firstinspires.ftc.teamcode.utils.units.Vec

@Config
internal object ODOMETRY_CONFIG {
    @JvmField
    var X_POD_POSITION = 0.0

    @JvmField
    var Y_POD_POSITION = 0.0

    @JvmField
    var HEADING_FILTER = 0.2

    @JvmField
    var ROBOT_SIZE = Vec(0.0, 0.0)
}

class Odometry(val pos: Pos, val linearVel: Vec, val angularVel: Double) {
    companion object {
        val ZERO
            get() = Odometry(Pos.ZERO, Vec.ZERO, 0.0)
    }
}

class GetOdometryConsumerEvent(var consumer: () -> Odometry = { Odometry.ZERO })

fun attachOdometry(collector: Collector) {
    val pinpoint = collector.hardwareMap.get("odometry") as GoBildaPinpointDriver

    pinpoint.setBulkReadScope(
        GoBildaPinpointDriver.Register.X_POSITION,
        GoBildaPinpointDriver.Register.Y_POSITION,
        GoBildaPinpointDriver.Register.X_VELOCITY,
        GoBildaPinpointDriver.Register.Y_VELOCITY,
        GoBildaPinpointDriver.Register.H_ORIENTATION,
        GoBildaPinpointDriver.Register.H_VELOCITY
    )

    pinpoint.setOffsets(
        ODOMETRY_CONFIG.X_POD_POSITION,
        ODOMETRY_CONFIG.Y_POD_POSITION,
        DistanceUnit.METER
    )

    val imu = collector.hardwareMap.get("imu") as IMU

    imu.initialize(
        IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        )
    )

    var odometry = Odometry.ZERO
    var atomicOdometry = odometry
    val odometryConsumer = { odometry }

    val headingFilter = ExponentialFilter(ODOMETRY_CONFIG.HEADING_FILTER)
    headingFilter.start()

    collector.eventBus.sub(GetOdometryConsumerEvent::class) {
        it.consumer = odometryConsumer
    }

    val scope = CoroutineScope(Dispatchers.IO)

    suspend fun CoroutineScope.updateOdometry() {
        var robotYaw = Ang.ZERO

        val updateImu = launch {
            robotYaw = Ang(imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))
        }

        var pinpointOdometry = Odometry.ZERO

        val updatePinpoint = launch {
            pinpoint.update()

            pinpointOdometry = Odometry(
                Pos(
                    pinpoint.getPosX(DistanceUnit.METER),
                    pinpoint.getPosY(DistanceUnit.METER),
                    pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)
                ),
                Vec(
                    pinpoint.getVelX(DistanceUnit.METER),
                    pinpoint.getVelY(DistanceUnit.METER)
                ),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
            )
        }

        updateImu.join()
        updatePinpoint.join()

        atomicOdometry = pinpointOdometry

        val filteredHeading = Ang.chop(
            headingFilter.updateRaw(
                pinpointOdometry.pos.angle(),
                (robotYaw - pinpointOdometry.pos.angle).angle
            )
        )

        pinpoint.setHeading(filteredHeading, AngleUnit.RADIANS)
    }

    var odometryJob = scope.launch { updateOdometry() }

    val update = {
        runBlocking { odometryJob.join() }

        odometry = atomicOdometry

        odometryJob = scope.launch { updateOdometry() }

        collector.telemetry.drawRect(
            atomicOdometry.pos.vec,
            ODOMETRY_CONFIG.ROBOT_SIZE,
            atomicOdometry.pos.angle(),
            Color.ORANGE
        )
        collector.telemetry.addData("robot pos", atomicOdometry.pos)

        headingFilter.k = ODOMETRY_CONFIG.HEADING_FILTER
    }

    collector.initUpdateEvent += update
    collector.updateEvent += update
}