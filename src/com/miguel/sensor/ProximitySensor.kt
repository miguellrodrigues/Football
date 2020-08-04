package com.miguel.sensor

import coppelia.BoolW
import coppelia.FloatWA

class ProximitySensor(val handle: Int) {

    var detectionState = BoolW(false)

    var detectedPoint = FloatWA(3)

    fun getState() : Boolean {
        return this.detectionState.value
    }

    private fun getDetectedPoint() : Array<Float> {
        return detectedPoint.array.toTypedArray()
    }
}