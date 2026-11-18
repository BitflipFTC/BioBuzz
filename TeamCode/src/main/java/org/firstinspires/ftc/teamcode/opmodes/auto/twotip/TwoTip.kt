package org.firstinspires.ftc.teamcode

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.Scheduler.schedule
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

    lateinit var start                         : Pose
    lateinit var gardenIntakeEnd               : Pose
    lateinit var driveToOtherHiveControl1      : Pose
    lateinit var otherHiveShoot                : Pose
    lateinit var lineupFlowertake              : Pose
    lateinit var flowerIntake                  : Pose
    lateinit var driveBackFromFlowertakeControl: Pose
    lateinit var parkPose                      : Pose

    lateinit var gate : Gate
    lateinit var shooter : Shooter
    lateinit var intake : Intake
    lateinit var flowertake : Flowertake
    lateinit var turret : Turret

    // Autonomous routine
    fun autoRoutine(): Command = sequential(
        instant {intake.on()},
        instant { shooter.setTargetState(turret.shooterDistance)},
        waitMs(2000.0),
        shoot(),
        follow(follower, gardenIntake()),
        waitMs(500.0),
        instant {shooter.targetFlywheelRPM = 3100.0},
        follow(follower, driveToOtherHive()),
        shoot(),
        instant { flowertake.down() },
        follow(follower, LineupFlowertake()),
        follow(follower, Flowerintake()).raceWith(waitMs(500.0)),
        waitMs(1500.0),
        follow(follower, drivebackagain()),
        instant { flowertake.up() },
        shoot(),
        follow(follower, parque())
    )

    fun shoot(): Command = sequential(
        instant{intake.transfer() },
        waitMs(250.0),
        instant {gate.open()},
        waitMs(1000.0),
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
        Turret.USE_SOTM = false
        follower = Constants.create(hardwareMap)

        val subsystems = listOf(gate, shooter, intake, flowertake, turret)

        if (alliance == Alliance.RED) {
            poseFactory.mirrorAroundPoint(70.75, 70.75)
        }

        val yStart  = 133.0
        val yHive   = 22.0
        val xFlower = 94.0
        val xHiveShoot = 106.0

        start                          = poseFactory.of( 83.0,  yStart,  0.0)
        gardenIntakeEnd                = poseFactory.of(131.5,  yStart,  0.0)
        driveToOtherHiveControl1       = poseFactory.of(xHiveShoot,  yStart,  0.0)
        otherHiveShoot                 = poseFactory.of(xHiveShoot,  yHive,   270.0)
        lineupFlowertake               = poseFactory.of(xFlower, yHive,   270.0)
        flowerIntake                   = poseFactory.of(xFlower,  11.5,   270.0)
        driveBackFromFlowertakeControl = poseFactory.of(xFlower, yHive,   0.0)
        parkPose                       = poseFactory.of(132.0,  20.0,   270.0)
        follower.setPose(start)
        follower.update()

        waitForStart()

        turret.alliance = alliance
        turret.updateRobotState(follower.pose(), follower.velocity())

        gate.close()

        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        turret.trackGoalCommand.schedule()
        shooter.targetFlywheelRPM = 3060.0

        schedule(autoRoutine())

        while (opModeIsActive()) {
            allHubs.forEach { hub -> hub.clearBulkCache() }
            follower.update()
            turret.updateRobotState(follower.pose(), follower.velocity())
            Scheduler.execute()

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

    fun gardenIntake(): Path = Paths.line(start, gardenIntakeEnd).linear(start, gardenIntakeEnd)

    fun driveToOtherHive(): Path = Paths.curve(gardenIntakeEnd, driveToOtherHiveControl1, otherHiveShoot).linear(gardenIntakeEnd, otherHiveShoot)

    fun LineupFlowertake(): Path = Paths.line(otherHiveShoot, lineupFlowertake).linear(otherHiveShoot, lineupFlowertake)

    fun Flowerintake(): Path = Paths.line(lineupFlowertake, flowerIntake).tangent()

    fun drivebackagain(): Path = Paths.curve(flowerIntake, driveBackFromFlowertakeControl, otherHiveShoot).linear(flowerIntake, otherHiveShoot)

    fun parque(): Path = Paths.line(otherHiveShoot, parkPose).constant(parkPose)
}
