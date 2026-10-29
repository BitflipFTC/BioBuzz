package org.firstinspires.ftc.teamcode

import club.bitflip.utils.TelemetryImplUpstreamSubmission
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.math.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.Full.Alliance
import org.firstinspires.ftc.teamcode.Full.Companion.alliance
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Flowertake
import org.firstinspires.ftc.teamcode.subsystems.Gate
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret

@Autonomous
class BlueAuto : LinearOpMode() {
        val factory = PoseFactory.degrees();
        val startPose = factory.of(59.0, 9.5, 90.0)
        val offWallPose = factory.of(59.0, 12.0, 90.0)
        val parkPose = factory.of(16.0, 104.0, 90.0)

    fun startToOffWall() = Paths.line(startPose, offWallPose).linear(startPose, parkPose)
    fun offWallToPark() = Paths.line(offWallPose, parkPose).linear(offWallPose, parkPose)

    override fun runOpMode() {
        Scheduler.reset()

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))

        val gate = Gate(hardwareMap, telemetry)
        val shooter = Shooter(hardwareMap, telemetry)
        val intake = Intake(hardwareMap, telemetry)
        val flowertake = Flowertake(hardwareMap, telemetry)
        val drivetrain = Drivetrain(hardwareMap, telemetry)
        val turret = Turret(hardwareMap, telemetry)
        val follower = Constants.create(hardwareMap)
        follower.setPose(startPose)
        follower.update()

        val subsystems = listOf(gate, shooter, intake, flowertake, drivetrain, turret)

        waitForStart()
        turret.alliance = Alliance.BLUE
//        turret.position = 0.5
        turret.robotPose = follower.pose()
//        turret.trackGoalCommand.schedule()

        gate.close()

        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        val loopTimer = LoopTimer(10)

        val auto = Groups.sequential(
            PedroCommands.follow(follower, startToOffWall()),
            instant {intake.on()},
            waitMs(1000.0),
            instant {gate.open()},
            waitMs(5000.0),
            instant{gate.close()},
            PedroCommands.follow(follower, offWallToPark())
        )

        waitForStart()

        turret.trackGoalCommand.schedule()
        auto.schedule()
        // flat against wall
        shooter.hoodPosition = 0.6
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
            Drawing.drawRobot(Pose2D(DistanceUnit.INCH, follower.pose().x(), follower.pose().y(), AngleUnit.RADIANS, follower.pose().heading()))
            telemetry.update()
        }
    }
}