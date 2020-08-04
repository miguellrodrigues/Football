package com.miguel.util

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

object Angle {

    fun normalizeRadian(radian: Double): Double {
        return atan2(sin(radian), cos(radian))
    }

    fun getCircumferencePoints(center: Vector, radius: Double): ArrayList<Vector> {
        val points = ArrayList<Vector>()

        var j = 0.0
        while (j < radius) {
            j += 0.1

            for (i in 0..360) {
                val radians = Math.toRadians(i.toDouble())

                val x = (j * cos(radians))
                val y = (j * sin(radians))

                val point = Vector(x, y, 0.0)

                center.add(point)

                points.add(center.clone())

                center.subtract(point)
            }
        }

        return points
    }
}