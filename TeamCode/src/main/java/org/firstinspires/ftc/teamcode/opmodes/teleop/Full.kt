package org.firstinspires.ftc.teamcode.opmodes.teleop

import club.bitflip.utils.TelemetryImplUpstreamSubmission
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.pedropathing.api.PoseFactory
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.follower.ManualDrive
import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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

@TeleOp(name = "buzz light-year")
class Full : LinearOpMode() {

    override fun runOpMode() {
        var fieldCentric = true;
        Scheduler.reset()

        val factory = PoseFactory.degrees()

        if (OpModeStorage.savedAlliance == Alliance.RED) {
            factory.mirrorAroundPoint(70.75, 70.75)
        }

        val startPos = factory.of(83.0, 132.0, 270.0)
        val oppositePos = factory.of(83.0, 9.5, 90.0)

        telemetry =
            JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))

        val gate = Gate(hardwareMap, telemetry)
        val shooter = Shooter(hardwareMap, telemetry)
        val intake = Intake(hardwareMap, telemetry)
        val flowertake = Flowertake(hardwareMap, telemetry)
        val turret = Turret(hardwareMap, telemetry)
        val follower = Constants.create(hardwareMap)

        follower.setPose(OpModeStorage.savedPose ?: startPos)
        Drawing.init()

        val subsystems = listOf(gate, shooter, intake, flowertake, turret)

        var alliance: Alliance? = OpModeStorage.savedAlliance

        while (opModeInInit()) {
            if (gamepad1.crossWasPressed()) {
                alliance = Alliance.BLUE
            }

            if (gamepad1.circleWasPressed()) {
                alliance = Alliance.RED
            }

            telemetry.run {
                addData("Initialized", true)
                addLine("Press Circle for red alliance, Cross for blue")
                addData("Current Alliance", alliance?.name ?: "Null")
                update()
            }

            gate.close()
        }

        turret.alliance = alliance
        turret.position = 0.5
        turret.robotPose = follower.pose()
//        turret.trackGoalCommand.schedule()

        gate.close()
        flowertake.up()
        gamepad1.triggerThreshold = 0.25f

        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        val loopTimer = LoopTimer(10)

        loopTimer.start()
        while (opModeIsActive()) {
            allHubs.forEach { hub -> hub.clearBulkCache() }

            loopTimer.end()
            loopTimer.start()

            val x = gamepad1.left_stick_x.toDouble()
            val y = -gamepad1.left_stick_y.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()

            if (gamepad1.touchpadWasPressed()) {
                fieldCentric = !fieldCentric
            }

            val drivePowers = if (fieldCentric) {
                ManualDrive.fieldCentric(
                    -y, x, -rx,
                    follower.pose().heading(),
                    if (OpModeStorage.savedAlliance == Alliance.RED) Math.PI else 0.0
                )
            } else {
                DrivePowers(-y, x, -rx)
            }

            follower.manual(drivePowers)

            if (gamepad1.rightBumperWasPressed()) {
                gate.toggle()
            }

            if (gamepad1.xWasPressed()) {
                intake.toggle()
            }

            if (gamepad1.cross) {
                intake.reversed = true
            } else {
                intake.reversed = false
            }

            if (gamepad1.bWasPressed()) {
                flowertake.toggle()
            }

            if (gamepad1.yWasPressed()) {
                // flat against wall
                shooter.targetFlywheelRPM = 3000.0
            }

            if (gamepad1.rightTriggerWasPressed()) {
                gate.open()
            }

            if (gamepad1.rightTriggerWasReleased()) {
                gate.close()
            }

            if (gamepad1.dpadUpWasPressed()) {
                shooter.targetFlywheelRPM += 125.0 / 2
            }

            if (gamepad1.dpadDownWasPressed()) {
                shooter.targetFlywheelRPM -= 125.0 / 2
            }

            if (gamepad1.left_trigger >= 0.25) {
                turret.trackGoal()
                shooter.setTargetState(turret.distanceToGoal)
            }

            if (gamepad1.startWasPressed()) {
                follower.setPose(startPos)
            }

            if (gamepad1.backWasPressed()) {
                follower.setPose(oppositePos)
            }

            turret.robotPose = follower.pose()
            turret.robotVelocity = follower.velocity()

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

        gate.close()
        flowertake.up()
    }
}