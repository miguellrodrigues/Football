package com.miguel.control

class Pid(private val kp: Double, private val ki: Double, private val kd: Double, private val saturation: Double, private val maxError: Double) {

    private var error = 0.0

    private var oldError = 0.0
    private var accumulator = 0.0
    private var out = 0.0

    fun update(error: Double, time: Double): Double {
        oldError = this.error
        this.error = error

        val proportional = kp * error
        val integral = ki * accumulator
        val derivative = kd * (error - oldError) / time

        if (saturation > out && out > -saturation) {
            if (maxError > 0) {
                if (error < maxError) {
                    accumulator += (error + oldError) / 2 * time
                }
            } else {
                accumulator += (error + oldError) / 2 * time
            }
        }

        val out = proportional + integral + derivative

        this.out = out

        return out
    }

    fun zero() {
        accumulator = 0.0
    }
}