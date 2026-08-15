package org.firstinspires.ftc.teamcode.collector

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.utils.units.Pos

enum class GameColor {
    RED,
    BLUE
}

enum class Side {
    FLOWER_FIELD,
    HIVE
}

enum class StartOrientation(val color: GameColor, val side: Side) {
    BLUE_HIVE(GameColor.BLUE, Side.HIVE),
    RED_HIVE(GameColor.RED, Side.HIVE),
    BLUE_FLOWER(GameColor.BLUE, Side.FLOWER_FIELD),
    RED_FLOWER(GameColor.RED, Side.FLOWER_FIELD)
}

object Settings {
    var orientation = StartOrientation.RED_HIVE
}