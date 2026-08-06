package org.firstinspires.ftc.teamcode.utils.units

class Pos(val vec: Vec, val angle: Ang) {
    companion object {
        val ZERO
            get() = Pos(Vec.ZERO, Ang.ZERO)
    }

    val x = vec.x

    var y = vec.y

    fun angle() = angle.angle

    constructor(x: Double) : this(Vec(x), Ang(x))
    constructor(x: Vec) : this(x, Ang.ZERO)
    constructor(x: Ang) : this(Vec.ZERO, x)
    constructor() : this(Vec.ZERO, Ang.ZERO)

    operator fun plus(pos: Pos) =
        Pos(vec + pos.vec, angle + pos.angle)

    operator fun plus(vec: Vec) = Pos(this.vec + vec, angle)
    operator fun plus(angle: Ang) = Pos(vec, this.angle + angle)

    operator fun minus(pos: Pos) =
        Pos(vec - pos.vec, angle - pos.angle)

    operator fun minus(vec: Vec) = Pos(this.vec - vec, angle)
    operator fun minus(angle: Ang) = Pos(vec, this.angle - angle)

    operator fun times(pos: Pos) =
        Pos(vec * pos.vec, angle * pos.angle)

    operator fun times(vec: Vec) = Pos(vec * this.vec, angle)
    operator fun times(angle: Ang) = Pos(vec, angle * this.angle)

    operator fun div(pos: Pos) =
        Pos(vec / pos.vec, angle / pos.angle)

    operator fun div(vec: Vec) = Pos(this.vec / vec, angle)
    operator fun div(angle: Ang) = Pos(vec, this.angle / angle)

    override fun toString(): String {
        return "[$vec, $angle]"
    }

    override fun equals(other: Any?): Boolean {
        if (other == null)
            return false

        if (other is Pos && other.vec == vec && other.angle == angle)
            return true

        return false
    }

    override fun hashCode(): Int {
        var result = angle.hashCode()
        result = 31 * result + x.hashCode()
        result = 31 * result + y.hashCode()
        return result
    }

    fun clone() = Pos(vec.clone(), angle.clone())
}