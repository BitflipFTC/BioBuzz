package org.firstinspires.ftc.teamcode

import club.bitflip.utils.TelemetryImplUpstreamSubmission
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Flowertake
import org.firstinspires.ftc.teamcode.subsystems.Gate
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret

@TeleOp(name = "buzz light-year")
class Full : LinearOpMode() {

    override fun runOpMode() {
        Scheduler.reset()

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))

        val gate = Gate(hardwareMap, telemetry)
        val shooter = Shooter(hardwareMap, telemetry)
        val intake = Intake(hardwareMap, telemetry)
        val flowertake = Flowertake(hardwareMap, telemetry)
        val drivetrain = Drivetrain(hardwareMap, telemetry)
        val turret = Turret(hardwareMap, telemetry)

        val subsystems = listOf(gate, shooter, intake, flowertake, drivetrain, turret)

        val shoot = waitUntil(shooter::readyToShoot).then(
            deadline(
                waitMs(3000.0),
                sequential(
                    instant { intake.mode = Intake.Mode.SHOOTING },
                    instant { gate.open() },
                )
            ).setEnd { gate.close() }
                .requiring(gate, shooter, intake)
        )


        telemetry.run {
            addData("Initialized", true)
            update()
        }

        waitForStart()

        turret.position = 0.5
        gate.close()

        while (opModeIsActive()) {

            turret.position = 0.49

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

            if (gamepad1.bWasPressed()) {
                flowertake.toggle()
            }

            if (gamepad1.yWasPressed()) {
                // flat against wall
                shooter.hoodPosition = 0.7
                shooter.targetFlywheelRPM = 3600.0
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

            subsystems.forEach { it.loop() }

            telemetry.update()
        }
    }
}