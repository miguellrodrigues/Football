package com.miguel

import com.miguel.control.Pid
import com.miguel.util.Angle
import com.miguel.util.Vector
import coppelia.*
import java.nio.channels.FileLock
import java.util.*
import kotlin.math.cos
import kotlin.math.sin
import kotlin.properties.Delegates
import kotlin.random.Random

object Main {

    private const val robot = "lumibot"

    enum class State {
        POSITION, KICK, WAIT
    }

    private var state = State.POSITION

    private val sim = remoteApi()

    private var clientId by Delegates.notNull<Int>()

    private fun sendCommand(command: String) {
        val options = StringWA(1)
        options.array[0] = command

        sim.simxCallScriptFunction(
                clientId,
                "lumibot",
                1,
                "processCommand",
                IntWA(0),
                FloatWA(0),
                options,
                CharWA(0),
                IntWA(0),
                FloatWA(0),
                StringWA(0),
                CharWA(0),
                remoteApi.simx_opmode_blocking
        )
    }

    private fun getSimulationData(parameter: String): Array<Float> {
        val options = StringWA(1)
        options.array[0] = parameter

        val out = FloatWA(1)

        sim.simxCallScriptFunction(
                clientId,
                "lumibot",
                1,
                "getSimulationData",
                IntWA(0),
                FloatWA(0),
                options,
                CharWA(0),
                IntWA(0),
                out,
                StringWA(0),
                CharWA(0),
                remoteApi.simx_opmode_blocking
        )

        return out.array.toTypedArray()
    }

    @JvmStatic
    fun main(args: Array<String>) {

        sim.simxFinish(-1)

        println("Conectando...")

        val clientId = sim.simxStart("127.0.0.1", 19999, true, true, 5000, 5)

        this.clientId = clientId

        if (clientId != -1) {
            println("Conectado com sucesso")

            val robotHandle = IntW(1)
            sim.simxGetObjectHandle(clientId, "${robot}_body", robotHandle, remoteApi.simx_opmode_blocking)

            val ballHandle = IntW(1)
            sim.simxGetObjectHandle(clientId, "ball", ballHandle, remoteApi.simx_opmode_blocking)

            val goalHandle = IntW(1)
            sim.simxGetObjectHandle(clientId, "goal", goalHandle, remoteApi.simx_opmode_blocking)

            val rightMotorHandle = IntW(1)
            sim.simxGetObjectHandle(clientId, "${robot}_rightMotor", rightMotorHandle, remoteApi.simx_opmode_blocking)

            val leftMotorHandle = IntW(1)
            sim.simxGetObjectHandle(clientId, "${robot}_leftMotor", leftMotorHandle, remoteApi.simx_opmode_blocking)

            val robotPos = FloatWA(3)
            sim.simxGetObjectPosition(clientId, robotHandle.value, -1, robotPos, remoteApi.simx_opmode_streaming)

            val ballPos = FloatWA(3)
            sim.simxGetObjectPosition(clientId, ballHandle.value, -1, ballPos, remoteApi.simx_opmode_streaming)

            val goalPos = FloatWA(3)
            sim.simxGetObjectPosition(clientId, goalHandle.value, -1, goalPos, remoteApi.simx_opmode_streaming)

            val robotOrientation = FloatWA(3)
            sim.simxGetObjectOrientation(clientId, robotHandle.value, -1, robotOrientation, remoteApi.simx_opmode_streaming)

            Thread.sleep(10)

            val running = true

            var rightVelocity = 0.0
            var leftVelocity = 0.0

            sim.simxStartSimulation(clientId, remoteApi.simx_opmode_oneshot)

            val radius = .67

            val anglePID = Pid(10.0, 0.0, 0.5, 4.0, Math.toRadians(1.0))
            val distancePID = Pid(8.0, 1.0, 0.0, 4.0, 5.0)

            var counter = getSimulationData("counter")[0].toInt()

            loop@ while (running) {
                sim.simxGetObjectPosition(clientId, robotHandle.value, -1, robotPos, remoteApi.simx_opmode_buffer)

                sim.simxGetObjectPosition(clientId, ballHandle.value, -1, ballPos, remoteApi.simx_opmode_buffer)

                sim.simxGetObjectPosition(clientId, goalHandle.value, -1, goalPos, remoteApi.simx_opmode_buffer)

                sim.simxGetObjectOrientation(clientId, robotHandle.value, -1, robotOrientation, remoteApi.simx_opmode_buffer)

                val robotVector = Vector(robotPos.array[0].toDouble(), robotPos.array[1].toDouble(), robotPos.array[2].toDouble())

                val ballVector = Vector(ballPos.array[0].toDouble(), ballPos.array[1].toDouble(), ballPos.array[2].toDouble())

                val goalPosition = Vector(goalPos.array[0].toDouble(), goalPos.array[1].toDouble(), 0.05)

                when (state) {
                    State.POSITION -> {
                        val theta = Angle.normalizeRadian(ballVector.differenceAngle(goalPosition))

                        val opposite = theta - Math.PI

                        val x = cos(opposite) * radius
                        val y = sin(opposite) * radius

                        val point = ballVector.clone()

                        point.add(Vector(x, y, 0.0))

                        val distance = robotVector.distance(point)

                        val angleOUT = anglePID.update(robotOrientation.array[2] - Angle.normalizeRadian(robotVector.differenceAngle(point)), 0.05)
                        val distanceOUT = distancePID.update(distance, 0.05)

                        rightVelocity = -angleOUT + distanceOUT
                        leftVelocity = angleOUT + distanceOUT

                        if (distance <= 0.01) {
                            state = State.KICK
                        }
                    }

                    State.KICK -> {
                        val theta = Angle.normalizeRadian(ballVector.differenceAngle(goalPosition))

                        val error = robotVector.distance(ballVector)

                        if (error < 0.108) {
                            state = State.WAIT
                        }

                        val angleOUT = anglePID.update(robotOrientation.array[2] - theta, 1.0)
                        val distanceOUT = distancePID.update(error,0.4)

                        rightVelocity = -angleOUT + distanceOUT
                        leftVelocity = angleOUT + distanceOUT
                    }

                    State.WAIT -> {
                        val score = getSimulationData("counter")[0].toInt()

                        rightVelocity = 0.0
                        leftVelocity = 0.0

                        if (score > counter) {
                            val teleport = FloatWA(3)

                            teleport.array[0] = Random.nextDouble(-1.8, 0.8).toFloat()
                            teleport.array[1] = Random.nextDouble(-1.8, 0.8).toFloat()
                            teleport.array[2] = 0.05.toFloat()

                            sim.simxSetObjectPosition(clientId, ballHandle.value, -1, teleport, remoteApi.simx_opmode_oneshot)

                            state = State.POSITION

                            counter = score
                        }
                    }
                }

                sim.simxPauseCommunication(clientId, true)
                sim.simxSetJointTargetVelocity(clientId, rightMotorHandle.value, rightVelocity.toFloat(), remoteApi.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientId, leftMotorHandle.value, leftVelocity.toFloat(), remoteApi.simx_opmode_oneshot)
                sim.simxPauseCommunication(clientId, false)
            }

            sim.simxFinish(clientId)

        } else {
            println("Ocorreu um erro ao se conectar")
        }
    }
}
