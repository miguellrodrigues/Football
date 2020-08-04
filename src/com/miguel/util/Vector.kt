package com.miguel.util

import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sqrt

class Vector(var x: Double, var y: Double, var z: Double) {

    fun add(vector: Vector): Vector {
        x += vector.x
        y += vector.y
        z += vector.z

        return this
    }

    fun subtract(vector: Vector): Vector {
        x -= vector.x
        y -= vector.y
        z -= vector.z

        return this
    }

    fun multiply(vector: Vector): Vector {
        x *= vector.x
        y *= vector.y
        z *= vector.z

        return this
    }

    fun divide(vector: Vector): Vector {
        x /= vector.x
        y /= vector.y
        z /= vector.z

        return this
    }

    fun length(): Double {
        return sqrt(
                x.pow(2.0) + y.pow(2.0) + z.pow(2.0)
        )
    }

    fun lengthSquared(): Double {
        return length().pow(2.0)
    }

    fun distance(vector: Vector): Double {
        return hypot(x - vector.x, y - vector.y)
    }

    fun distanceSquared(vector: Vector): Double {
        return distance(vector).pow(2.0)
    }

    fun differenceAngle(vector: Vector): Double {
        return atan2(vector.y - y, vector.x - x)
    }

    fun midPoint(vector: Vector): Vector {
        x = (x + vector.x) / 2
        y = (y + vector.y) / 2
        z = (z + vector.z) / 2

        return this
    }

    fun getMidPoint(vector: Vector): Vector {
        x = (x + vector.x) / 2
        y = (y + vector.y) / 2
        z = (z + vector.z) / 2

        return Vector(x, y, z)
    }

    fun multiply(value: Double): Vector {
        x *= value
        y *= value
        z *= value

        return this
    }

    fun divide(value: Double): Vector {
        x /= value
        y /= value
        z /= value

        return this
    }

    fun normalize(): Vector {
        val length = length()
        x /= length
        y /= length
        z /= length

        return this
    }

    fun clone(): Vector {
        return Vector(x, y, z)
    }

    override fun toString(): String {
        return "Vector(x=$x, y=$y, z=$z)"
    }
}