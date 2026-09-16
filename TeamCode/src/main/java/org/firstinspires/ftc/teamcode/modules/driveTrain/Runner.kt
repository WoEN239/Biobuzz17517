package org.firstinspires.ftc.teamcode.modules.driveTrain

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.utils.Reg
import org.firstinspires.ftc.teamcode.utils.RegParams
import org.firstinspires.ftc.teamcode.utils.units.Ang
import org.firstinspires.ftc.teamcode.utils.units.Pos
import org.firstinspires.ftc.teamcode.utils.units.Vec
import java.lang.Math.toRadians
import java.util.LinkedList
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.math.withSign

@Config
internal object RUNNER_CONFIG {
    @JvmField
    var POSITION_WINDOW = 0.15

    @JvmField
    var ANGULAR_WINDOW = toRadians(25.0)

    @JvmField
    var POS_SENTRY_TIMER = 5.0

    @JvmField
    var ANGULAR_SENTRY_TIMER = 5.0

    @JvmField
    var X_REGULATOR = RegParams()

    @JvmField
    var Y_REGULATOR = RegParams()

    @JvmField
    var H_REGULATOR = RegParams()

    @JvmField
    var TARGET_TIMER = 0.01
}

interface ITrajectorySegment

data class TurnSegment(
    val endHeading: Ang,
    val velConstrain: Double? = null,
    val angularWindow: Double? = null,
    val sentryTimer: Double? = null
) : ITrajectorySegment

data class MoveSegment(
    val endPoint: Vec,
    val velConstrain: Double? = null,
    val positionWindow: Double? = null,
    val sentryTimer: Double? = null
) : ITrajectorySegment

data class DriveSegment(
    val endPos: Pos,
    val linearVelConstrain: Double? = null,
    val angularVelConstrain: Double? = null,
    val positionWindow: Double? = null,
    val angularWindow: Double? = null,
    val sentryTimer: Double? = null
) : ITrajectorySegment

class RunSegmentEvent(val segment: ITrajectorySegment)
class GetRunnerAtTargetPositionEvent(var atTarget: () -> Boolean = { true })
class GetRunnerAtTargetAngleEvent(var atTarget: () -> Boolean = { true })

fun attachRunner(collector: Collector) {
    val segments = LinkedList<ITrajectorySegment>()

    val xRegulator = Reg(RUNNER_CONFIG.X_REGULATOR)
    val yRegulator = Reg(RUNNER_CONFIG.Y_REGULATOR)
    val hRegulator = Reg(RUNNER_CONFIG.H_REGULATOR)

    var linearVelConstrain: Double? = null
    var angularVelConstrain: Double? = null

    val posTargetTimer = ElapsedTime()
    val angularTargetTimer = ElapsedTime()

    val posSentryTimer = ElapsedTime()
    val angularSentryTimer = ElapsedTime()

    var targetPos = Pos.ZERO

    var atTargetRot = true
    var atTargetPos = true

    val atTargetRotConsumer = { atTargetRot }
    val atTargetPosConsumer = { atTargetPos }

    collector.eventBus.sub(GetRunnerAtTargetPositionEvent::class) {
        it.atTarget = atTargetPosConsumer
    }

    collector.eventBus.sub(GetRunnerAtTargetAngleEvent::class) {
        it.atTarget = atTargetRotConsumer
    }

    var posWindow = RUNNER_CONFIG.POSITION_WINDOW
    var angularWindow = RUNNER_CONFIG.ANGULAR_WINDOW

    var posSentry = RUNNER_CONFIG.POS_SENTRY_TIMER
    var angularSentry = RUNNER_CONFIG.ANGULAR_SENTRY_TIMER

    val odometryConsumer = collector.eventBus(GetOdometryConsumerEvent()).consumer

    collector.startEvent += {
        xRegulator.start()
        yRegulator.start()
        hRegulator.start()

        targetPos = odometryConsumer().pos
    }

    fun updateTarget() {
        if (segments.isNotEmpty()) {
            val first = segments.first()

            if (first is TurnSegment && atTargetRot) {
                atTargetRot = false

                segments.removeFirst()
                targetPos = Pos(targetPos.vec, first.endHeading)

                angularSentryTimer.reset()

                angularSentry = first.sentryTimer ?: RUNNER_CONFIG.ANGULAR_SENTRY_TIMER
                angularVelConstrain = first.velConstrain
                angularWindow = first.angularWindow ?: RUNNER_CONFIG.ANGULAR_WINDOW
            }

            if (first is MoveSegment && atTargetPos) {
                atTargetPos = false

                segments.removeFirst()
                targetPos = Pos(first.endPoint, targetPos.angle)

                posSentryTimer.reset()

                posSentry = first.sentryTimer ?: RUNNER_CONFIG.POS_SENTRY_TIMER
                linearVelConstrain = first.velConstrain
                posWindow = first.positionWindow ?: RUNNER_CONFIG.POSITION_WINDOW
            }

            if (first is DriveSegment && atTargetRot && atTargetPos) {
                atTargetRot = false
                atTargetPos = false

                segments.removeFirst()
                targetPos = first.endPos

                angularSentryTimer.reset()
                posSentryTimer.reset()

                angularSentry = first.sentryTimer ?: RUNNER_CONFIG.ANGULAR_SENTRY_TIMER
                angularVelConstrain = first.angularVelConstrain
                angularWindow = first.angularWindow ?: RUNNER_CONFIG.ANGULAR_WINDOW

                posSentry = first.sentryTimer ?: RUNNER_CONFIG.POS_SENTRY_TIMER
                linearVelConstrain = first.linearVelConstrain
                posWindow = first.positionWindow ?: RUNNER_CONFIG.POSITION_WINDOW
            }
        }
    }

    collector.eventBus.sub(RunSegmentEvent::class) {
        segments.add(it.segment)

        updateTarget()
    }

    collector.updateEvent += {
        val odometry = odometryConsumer()

        val err = targetPos - odometry.pos
        val localErr = err.vec.turn(-odometry.pos.angle())

        var linearVel = Vec(
            xRegulator
                .update(localErr.x, 0.0, collector.battery.currentVoltage),
            yRegulator
                .update(localErr.y, 0.0, collector.battery.currentVoltage)
        )

        var angularVel = hRegulator
            .update(err.angle(), 0.0, collector.battery.currentVoltage)

        linearVelConstrain?.let {
            if (linearVel.l() > it) {
                val l = linearVel.l()

                val k = it / l

                linearVel *= k
            }
        }

        angularVelConstrain?.let {
            if (angularVel.absoluteValue > it)
                angularVel = it.withSign(angularVel.sign)
        }

        collector.eventBus(SetDriveVelEvent(linearVel, angularVel))

        if (err.vec.l() < posWindow)
            atTargetPos = posTargetTimer.seconds() > RUNNER_CONFIG.TARGET_TIMER
        else {
            atTargetPos = false
            posTargetTimer.reset()

            if (posSentryTimer.seconds() > posSentry)
                atTargetPos = true
        }

        if (err.angle().absoluteValue < angularWindow)
            atTargetRot = angularTargetTimer.seconds() > RUNNER_CONFIG.TARGET_TIMER
        else {
            atTargetRot = false
            angularTargetTimer.reset()

            if (angularSentryTimer.seconds() > angularSentry)
                atTargetRot = true
        }

        updateTarget()
    }
}