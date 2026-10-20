package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.ServoEx
import club.bitflip.utils.normalizeRadians
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Full
import kotlin.math.atan2

class Turret(hardwareMap: HardwareMap, telemetry: Telemetry) : Subsystem(hardwareMap, telemetry) {
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

    var alliance: Full.Alliance? = null
        set (newAlliance) {
            field = newAlliance

            if (newAlliance == Full.Alliance.BLUE) {
                audienceHive = Pose(Hive.BLUE_AUDIENCE.x, Hive.BLUE_AUDIENCE.y)
                notAudienceHive = Pose(Hive.BLUE_NOT_AUDIENCE.x, Hive.BLUE_NOT_AUDIENCE.y)
            } else if (newAlliance == Full.Alliance.RED) {
                audienceHive = Pose(Hive.RED_AUDIENCE.x, Hive.RED_AUDIENCE.y)
                notAudienceHive = Pose(Hive.RED_NOT_AUDIENCE.x, Hive.RED_NOT_AUDIENCE.y)
            }
        }

    var robotPose = Pose(0.0,0.0,0.0)
    val turretPose: Pose
        get() = robotPose

    var position = 0.5
        set(newPosition) {
            field = newPosition
            turret1.position = newPosition
            turret2.position = newPosition
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

            position = (((clippedAngle) / (2 * 160)) + 0.5).coerceIn(0.0, 1.0)
        }

    val trackGoalCommand: Command = Commands.infinite(::trackGoal)

    override fun loop () {
        telemetry.addData("Turret angle", degrees)
    }

    fun trackGoal() {
        val goalPose = if (robotPose.y() >= 72) notAudienceHive else audienceHive

        val bearing = Math.toDegrees(atan2(goalPose.y() - turretPose.y(), goalPose.x() - turretPose.x()))

        // normalize robot pose between (-180, 180]
        val robotHeading = Math.toDegrees(normalizeRadians(robotPose.heading(), false))

        degrees = bearing - robotHeading
    }
}