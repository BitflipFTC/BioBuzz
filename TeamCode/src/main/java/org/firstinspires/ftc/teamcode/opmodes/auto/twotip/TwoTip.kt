package org.firstinspires.ftc.teamcode

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.Scheduler.schedule
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.commands.Commands.*
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.Flowertake
import org.firstinspires.ftc.teamcode.subsystems.Gate
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret

open class TwoTip(val alliance: Alliance) : LinearOpMode() {

    private lateinit var follower: Follower

    private val poseFactory = PoseFactory.degrees()

    lateinit var start         : Pose
    lateinit var leave         : Pose
    lateinit var point2Start   : Pose
    lateinit var point2        : Pose
    lateinit var point2Control1: Pose
    lateinit var point3Start   : Pose
    lateinit var point3        : Pose
    lateinit var point4        : Pose
    lateinit var point5Start   : Pose
    lateinit var point5        : Pose
    lateinit var point5Control1: Pose
    lateinit var point6        : Pose

    lateinit var gate : Gate
    lateinit var shooter : Shooter
    lateinit var intake : Intake
    lateinit var flowertake : Flowertake
    lateinit var turret : Turret

    // Autonomous routine
    fun autoRoutine(): Command = sequential(
        instant {intake.on()},
        waitMs(2000.0),
        shoot(),
        follow(follower, gardenIntake()),
        waitMs(500.0),
        instant {shooter.targetFlywheelRPM = 3125.0},
        follow(follower, driveToOtherHive()),
        shoot(),
        instant { flowertake.down() },
        follow(follower, LineupFlowertake()),
        follow(follower, Flowerintake()).raceWith(waitMs(2000.0)),
        waitMs(2000.0),
        follow(follower, drivebackagain()),
        instant { flowertake.up() },
        shoot(),
        follow(follower, parque())
    )

    fun shoot(): Command = sequential(
        instant{intake.transfer() },
        waitMs(250.0),
        instant {gate.open()},
        waitMs(2000.0),
        instant{gate.close()},
        instant {intake.on()}
    )

    override fun runOpMode() {
        Scheduler.reset()
        gate = Gate(hardwareMap, telemetry)
        shooter = Shooter(hardwareMap, telemetry)
        intake = Intake(hardwareMap, telemetry)
        flowertake = Flowertake(hardwareMap, telemetry)
        turret = Turret(hardwareMap, telemetry)
        follower = Constants.create(hardwareMap)

        val subsystems = listOf(gate, shooter, intake, flowertake, turret)


        if (alliance == Alliance.RED) {
            poseFactory.mirrorAroundPoint(70.75, 70.75)
        }

        start          = poseFactory.of(83.0, 133.0, 0.0)
        leave          = poseFactory.of(131.0, 133.0, 0.0)
        point2Start    = poseFactory.of(131.0, 133.0, 0.0)
        point2         = poseFactory.of(118.0, 22.0, 270.0)
        point2Control1 = poseFactory.of(112.0, 133.0, 0.0)
        point3Start    = poseFactory.of(118.0, 22.0, 270.0)
        point3         = poseFactory.of(94.0, 22.0, 270.0)
        point4         = poseFactory.of(94.0, 11.5, -90.0)
        point5Start    = poseFactory.of(94.0, 11.5, 270.0)
        point5         = poseFactory.of(118.0, 22.0, 270.0)
        point5Control1 = poseFactory.of(94.0, 22.0, 0.0)
        point6         = poseFactory.of(132.0, 22.0, 270.0)
        follower.setPose(start)
        follower.update()

        waitForStart()

        turret.alliance = alliance
        turret.robotPose = follower.pose()

        gate.close()

        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        turret.trackGoalCommand.schedule()
        shooter.targetFlywheelRPM = 3060.0

        schedule(autoRoutine())

        while (opModeIsActive()) {
            allHubs.forEach { hub -> hub.clearBulkCache() }
            follower.update()
            Scheduler.execute()

            turret.robotPose = follower.pose()
            subsystems.forEach { it.loop() }


            telemetry.addData("x", follower.pose().x())
            telemetry.addData("y", follower.pose().y())
            telemetry.addData("heading", follower.pose().heading())

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint())
                telemetry.addData("Path number", follower.pathIndex())
            }

            telemetry.update()
        }

        OpModeStorage.savedPose = follower.pose()
    }

    fun gardenIntake(): Path = Paths.line(start, leave).linear(start, leave)

    fun driveToOtherHive(): Path = Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2)

    fun LineupFlowertake(): Path = Paths.line(point3Start, point3).linear(point3Start, point3)

    fun Flowerintake(): Path = Paths.line(point3, point4).tangent()

    fun drivebackagain(): Path = Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5)

    fun parque(): Path = Paths.line(point5, point6).constant(point6)
}
