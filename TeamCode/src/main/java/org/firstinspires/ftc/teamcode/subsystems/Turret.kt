package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.OpModeManager
import club.bitflip.utils.hardware.ServoEx
import club.bitflip.utils.normalizeDegrees
import club.bitflip.utils.normalizeRadians
import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Alliance
import kotlin.math.atan2

@Configurable
class Turret(hardwareMap: HardwareMap, telemetry: Telemetry) : Subsystem(hardwareMap, telemetry) {
    companion object {
        @JvmField
        var SHOT_TIME = 0.5
        @JvmField
        var USE_SOTM = false
    }

    private val turret1 = ServoEx("turret1")
    private val turret2 = ServoEx("turret2")

    enum class Hive (val x: Double, val y: Double) {
        RED_AUDIENCE (58.0, 52.0),
        RED_NOT_AUDIENCE (58.0, 92.0),
        BLUE_AUDIENCE(84.0, 52.0),
        BLUE_NOT_AUDIENCE(84.0, 92.0)
    }

    lateinit var audienceHive: Pose
    lateinit var notAudienceHive: Pose

    var alliance: Alliance? = null
        set (newAlliance) {
            field = newAlliance

            if (newAlliance == Alliance.BLUE) {
                audienceHive = Pose(Hive.BLUE_AUDIENCE.x, Hive.BLUE_AUDIENCE.y)
                notAudienceHive = Pose(Hive.BLUE_NOT_AUDIENCE.x, Hive.BLUE_NOT_AUDIENCE.y)
            } else if (newAlliance == Alliance.RED) {
                audienceHive = Pose(Hive.RED_AUDIENCE.x, Hive.RED_AUDIENCE.y)
                notAudienceHive = Pose(Hive.RED_NOT_AUDIENCE.x, Hive.RED_NOT_AUDIENCE.y)
            }
        }

    var robotPose = Pose(0.0,0.0,0.0)
    var robotVelocity = Velocity(0.0,0.0,0.0)

    var distanceToGoal = 32.0

    val turretPose: Pose
        get() = robotPose

    var position = 0.0
        set(newPosition) {
            turret1.position = newPosition
            turret2.position = newPosition
            field = newPosition
        }

    var degrees: Double
        get() {
            return (position - 0.5) * (2 * 160)
        }
        set(newDegrees) {
            var clippedAngle = newDegrees

            while(clippedAngle < -180) {
                clippedAngle += 360.0
            }
            while(clippedAngle > 180) {
                clippedAngle -= 360.0
            }

            position = (((clippedAngle) / (2 * 160)) + 0.5).coerceIn(0.05, 0.95) //todo wiring
        }

    val trackGoalCommand: Command = Commands.infinite(::trackGoal)

    override fun loop () {
        telemetry.addData("Turret angle", degrees)
        telemetry.addData("distance to goal", distanceToGoal)
    }

    fun trackGoal() {
        val goalPose = if (robotPose.y() >= 72) notAudienceHive else audienceHive

        if (USE_SOTM) {
            val projectedRobotPose = robotPose.plus(robotVelocity.scale(SHOT_TIME).toPose())

            val bearing = normalizeDegrees(Math.toDegrees(atan2(goalPose.y() - projectedRobotPose.y(), goalPose.x() - projectedRobotPose.x())) + 180, false)

            // normalize robot pose between (-180, 180]
            val robotHeading = Math.toDegrees(normalizeRadians((projectedRobotPose.heading()), false))

            degrees = bearing - robotHeading
            distanceToGoal = goalPose.distance(projectedRobotPose)
        } else {
            val bearing = normalizeDegrees(
                Math.toDegrees(
                    atan2(
                        goalPose.y() - turretPose.y(),
                        goalPose.x() - turretPose.x()
                    )
                ) + 180, false
            )

            // normalize robot pose between (-180, 180]
            val robotHeading = Math.toDegrees(
                normalizeRadians(
                    (robotPose.heading()),
                    false
                )
            )

            degrees = bearing - robotHeading
            distanceToGoal = goalPose.distance(robotPose)

        }
    }

    fun Velocity.toPose(): Pose {
        return Pose(this.vx, this.vy, this.omega)
    }
}