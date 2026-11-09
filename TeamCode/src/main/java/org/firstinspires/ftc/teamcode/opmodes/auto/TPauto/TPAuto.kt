package org.firstinspires.ftc.teamcode.opmodes.auto.TPauto

import club.bitflip.utils.TelemetryImplUpstreamSubmission
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.Drawing
import org.firstinspires.ftc.teamcode.OpModeStorage
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Flowertake
import org.firstinspires.ftc.teamcode.subsystems.Gate
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret

open class TPAuto(
    val alliance: Alliance
) : LinearOpMode() {

    val factory = PoseFactory.degrees();

    lateinit var startPose: Pose
    lateinit var offWallPose: Pose
    lateinit var parkPose: Pose

    fun startToOffWall(): Path = Paths.line(startPose, offWallPose).constant(startPose)
    fun offWallToPark(): Path = Paths.curve(offWallPose, Pose(offWallPose.y(), parkPose.x()), parkPose).tangent()

    override fun runOpMode() {
        Scheduler.reset()

        OpModeStorage.savedAlliance = alliance

        if (alliance == Alliance.RED) {
            factory.mirrorAroundPoint(70.75, 70.75)
        }

            startPose = factory.of(83.0, 133.0, 0.0)
            offWallPose = factory.of(83.0, 128.0, 0.0)
            parkPose = factory.of(128.0, 47.0, 90.0)


        telemetry =
            JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))

        val gate = Gate(hardwareMap, telemetry)
        val shooter = Shooter(hardwareMap, telemetry)
        val intake = Intake(hardwareMap, telemetry)
        val flowertake = Flowertake(hardwareMap, telemetry)
        val turret = Turret(hardwareMap, telemetry)
        val follower = Constants.create(hardwareMap)
        follower.setPose(startPose)
        follower.update()

        val subsystems = listOf(gate, shooter, intake, flowertake, turret)

        waitForStart()
        turret.alliance = alliance
        turret.robotPose = follower.pose()

        gate.close()

        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        val loopTimer = LoopTimer(10)

        val auto = Groups.sequential(
            PedroCommands.follow(follower, startToOffWall()),
            Commands.instant { intake.transfer() },
            Commands.waitMs(1000.0),
            Commands.instant { gate.open() },
            Commands.waitMs(5000.0),
            Commands.instant { gate.close() },
            PedroCommands.follow(follower, offWallToPark()),
            Commands.instant { intake.off() },
            Commands.instant { shooter.targetFlywheelRPM = 0.0 }
        )

        waitForStart()

        auto.schedule()
        turret.trackGoalCommand.schedule()
        // flat against wall
        shooter.targetFlywheelRPM = 3000.0

        loopTimer.start()
        while (opModeIsActive()) {
            allHubs.forEach { hub -> hub.clearBulkCache() }

            loopTimer.end()
            loopTimer.start()

            turret.robotPose = follower.pose()

            Scheduler.execute()

            subsystems.forEach { it.loop() }

            follower.update()

            telemetry.addLine()
            telemetry.addData("Robot pos", "x: %.2f, y: %.2f, h: %.2f", follower.pose().x(), follower.pose().y(), follower.pose().heading())
            telemetry.addData("Loop ms", "%05.2f", loopTimer.ms.toDouble())
            Drawing.drawRobot(
                Pose2D(
                    DistanceUnit.INCH,
                    follower.pose().x(),
                    follower.pose().y(),
                    AngleUnit.RADIANS,
                    follower.pose().heading()
                )
            )
            telemetry.update()
        }

        OpModeStorage.savedPose = follower.pose()
    }
}