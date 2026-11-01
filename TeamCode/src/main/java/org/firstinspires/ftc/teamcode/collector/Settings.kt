package org.firstinspires.ftc.teamcode.collector

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.utils.units.Pos

enum class StartOrientation {
    LEFT,
    RIGHT
}

object Settings {
    var orientation = StartOrientation.LEFT
}