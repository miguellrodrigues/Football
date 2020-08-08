package com.miguel

import com.miguel.control.Pid
import com.miguel.util.Angle
import com.miguel.util.Vector
import coppelia.*
import kotlin.math.abs
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


            val robotOrientation = FloatWA(3)
            sim.simxGetObjectOrientation(clientId, robotHandle.value, -1, robotOrientation, remoteApi.simx_opmode_streaming)

            Thread.sleep(100)

            val running = true

            var rightVelocity: Double
            var leftVelocity: Double

            val radius = .75

            val anglePID = Pid(5.0, .0,0.75, 8.0, .0)
            val distancePID = Pid(3.0, .3, .0, 8.0, 1.0)

            var distance = .0
            var theta = .0

            var counter = .0
            var error = .0

            var lastBallPosition = Vector(.0, .0, .0)

            sim.simxStartSimulation(clientId, remoteApi.simx_opmode_oneshot)

            loop@ while (running) {
                sim.simxGetObjectPosition(clientId, robotHandle.value, -1, robotPos, remoteApi.simx_opmode_buffer)

                sim.simxGetObjectPosition(clientId, ballHandle.value, -1, ballPos, remoteApi.simx_opmode_buffer)

                sim.simxGetObjectPosition(clientId, goalHandle.value, -1, goalPos, remoteApi.simx_opmode_streaming)

                sim.simxGetObjectOrientation(clientId, robotHandle.value, -1, robotOrientation, remoteApi.simx_opmode_buffer)

                val robotVector = Vector(robotPos.array[0].toDouble(), robotPos.array[1].toDouble(), robotPos.array[2].toDouble())

                val ballVector = Vector(ballPos.array[0].toDouble(), ballPos.array[1].toDouble(), ballPos.array[2].toDouble())

                val goalPosition = Vector(goalPos.array[0].toDouble(), goalPos.array[1].toDouble(), .05)

                when (state) {
                    State.POSITION -> {
                        val directionAngle = ballVector.differenceAngle(goalPosition)

                        val opposite = directionAngle - Math.PI

                        val x = cos(opposite) * radius
                        val y = sin(opposite) * radius

                        val point = ballVector.clone()

                        point.add(Vector(x, y, .0))

                        distance = robotVector.distance(point)

                        theta = robotVector.differenceAngle(point)

                        if (distance <= .02) {
                            state = State.KICK

                            lastBallPosition = ballVector

                            distancePID.zero()
                        }
                    }

                    State.KICK -> {
                        distance = robotVector.distance(ballVector)

                        theta = ballVector.differenceAngle(goalPosition)

                        if (ballVector.distance(lastBallPosition) >= .1) {
                            state = State.POSITION
                        }

                        if (distance <= .108) {
                            state = State.WAIT
                        }
                    }

                    State.WAIT -> {
                        val score = getSimulationData("counter")[0].toDouble()

                        if (score > counter || ballVector.z < 0) {
                            val teleport = FloatWA(3)

                            var teleportVector = Vector(Random.nextDouble(-1.45, 1.45), Random.nextDouble(-.7250, .5250), 0.05)

                            while (teleportVector.distance(robotVector) <= radius) {
                                teleportVector = Vector(Random.nextDouble(-1.45, 1.45), Random.nextDouble(-.7250, .5250), 0.05)
                            }

                            teleport.array[0] = teleportVector.x.toFloat()
                            teleport.array[1] = teleportVector.y.toFloat()
                            teleport.array[2] = teleportVector.z.toFloat()

                            sim.simxSetObjectPosition(clientId, ballHandle.value, -1, teleport, remoteApi.simx_opmode_oneshot)

                            state = State.POSITION

                            if (ballVector.z < 0) {
                                error++
                            } else {
                                counter = score
                            }

                            val errorPercent = (error / (counter + error)) * 100.0

                            sim.simxAddStatusbarMessage(clientId, "Total de tentativas: ${(counter + error)} Porcentagem de erro: $errorPercent%", remoteApi.simx_opmode_oneshot)
                        }
                    }
                }

                if (state == State.WAIT) {
                    rightVelocity = .0
                    leftVelocity = .0
                } else {
                    val angleOUT = anglePID.update(Angle.normalizeRadian(robotOrientation.array[2] - theta), .05)
                    val distanceOUT = distancePID.update(distance, .05)

                    rightVelocity = distanceOUT - angleOUT
                    leftVelocity = distanceOUT + angleOUT
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
