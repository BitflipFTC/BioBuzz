package org.firstinspires.ftc.teamcode

import club.bitflip.utils.TelemetryImplUpstreamSubmission
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.behaviors.InterruptedBehavior
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Flowertake
import org.firstinspires.ftc.teamcode.subsystems.Gate
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret

@TeleOp(name = "buzz light-year")
class Full : LinearOpMode() {
    enum class Alliance {
        RED,
        BLUE
    }

    companion object {
        var alliance: Alliance? = null
    }

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
        follower.setPose(Pose(83.0, 132.0, Math.toRadians(270.0)))
        Drawing.init()

        val subsystems = listOf(gate, shooter, intake, flowertake, drivetrain, turret)

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

            shooter.hoodPosition = 0.5
            gate.close()
        }

        turret.alliance = alliance
//        turret.position = 0.5
        turret.robotPose = follower.pose()
//        turret.trackGoalCommand.schedule()

        gate.close()
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

            drivetrain.updateDrivetrain(x, y, rx)

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
                shooter.hoodPosition = 0.6
                shooter.targetFlywheelRPM = 3000.0
            }

            if (gamepad1.rightTriggerWasPressed()) {
                gate.open()
            }

            if (gamepad1.rightTriggerWasReleased()) {
                gate.close()
            }

            if (gamepad1.dpadUpWasPressed()) {
                shooter.targetFlywheelRPM += 125.0
            }

            if (gamepad1.dpadDownWasPressed()) {
                shooter.targetFlywheelRPM -= 125.0
            }

            if (gamepad1.dpadRightWasPressed()) {
                shooter.hoodPosition += 0.025
            }

            if (gamepad1.dpadLeftWasPressed()) {
                shooter.hoodPosition -= 0.025
            }

            if (gamepad1.left_trigger >= 0.25) {
                turret.trackGoal()
            }

            if (gamepad1.startWasPressed()) {
                follower.setPose(Pose(83.0, 132.0, Math.toRadians(270.0)))
            }

            if (gamepad1.backWasPressed()) {
                follower.setPose(Pose(83.0, 9.5, Math.toRadians(90.0)))
            }

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