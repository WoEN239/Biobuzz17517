package org.firstinspires.ftc.teamcode.modules.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.collector.Collector
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Vec

class Telemetry {
    private val _mobileTelemetry: Telemetry
    private val _dashboard: FtcDashboard

    private var _telemetryPacket = TelemetryPacket()

    private val _canvas
        get() = _telemetryPacket.fieldOverlay()

    constructor(collector: Collector) {
        _mobileTelemetry = collector.opMode.telemetry
        _dashboard = FtcDashboard.getInstance()

        fun update () {
            _dashboard.sendTelemetryPacket(_telemetryPacket)
            _telemetryPacket = TelemetryPacket()

            _mobileTelemetry.update()
        }

        collector.updateEvent += ::update
        collector.initUpdateEvent += ::update
    }

    fun addData(name: String, data: Any) {
        _mobileTelemetry.addData(name, data)
        _telemetryPacket.put(name, data)
    }

    fun addLine(line: String) {
        _mobileTelemetry.addLine(line)
        _telemetryPacket.addLine(line)
    }

    fun drawCircle(pos: Vec, radius: Double, color: String) {
        _canvas.setFill(color)
        _canvas.fillCircle(
            DistanceUnit.INCH.fromMeters(pos.x),
            DistanceUnit.INCH.fromMeters(pos.y),
            DistanceUnit.INCH.fromMeters(radius)
        )
    }

    fun drawCircle(pos: Vec, radius: Double, color: Color) =
        drawCircle(pos, radius, color.toHexString())

    fun drawPolygon(points: Array<Vec>, color: String) {
        val inchX = DoubleArray(points.size) { DistanceUnit.INCH.fromMeters(points[it].x) }
        val inchY = DoubleArray(points.size) { DistanceUnit.INCH.fromMeters(points[it].y) }

        _canvas.setFill(color)
        _canvas.fillPolygon(inchX, inchY)
    }

    fun drawPolygon(points: Array<Vec>, color: Color) = drawPolygon(points, color.toHexString())

    fun drawRect(center: Vec, size: Vec, rot: Double = 0.0, color: String) = drawPolygon(
        arrayOf(
            center + Vec(-size.x / 2, size.y / 2).turn(rot),
            center + Vec(size.x / 2, size.y / 2).turn(rot),
            center + Vec(size.x / 2, -size.y / 2).turn(rot),
            center + Vec(-size.x / 2, -size.y / 2).turn(rot)
        ), color
    )

    fun drawRect(center: Vec, size: Vec, rot: Double = 0.0, color: Color) =
        drawRect(center, size, rot, color.toHexString())

    fun log(vararg msg: String) {
        for (s in msg)
            logWithTag(s, "17517robot")
    }

    fun logWithTag(str: String, tag: String) =
        RobotLog.dd(tag, str)
}