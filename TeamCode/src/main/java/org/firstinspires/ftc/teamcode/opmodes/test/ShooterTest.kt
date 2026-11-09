package org.firstinspires.ftc.teamcode.opmodes.test

import club.bitflip.utils.TelemetryImplUpstreamSubmission
import club.bitflip.utils.hardware.MotorEx
import club.bitflip.utils.hardware.ServoEx
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.Utility
import org.firstinspires.ftc.teamcode.subsystems.Shooter

@Utility(name = "Tuning shooter")
class ShooterTest : LinearOpMode() {
    override fun runOpMode() {

        telemetry =
            JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))
        val shooter = Shooter(hardwareMap, telemetry)

        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                shooter.targetFlywheelRPM += 125.0
            }

            if (gamepad1.dpadDownWasPressed()) {
                shooter.targetFlywheelRPM -= 125.0
            }

            if (gamepad1.aWasPressed()) {
                shooter.toggleBangBang()
            }

            shooter.loop()

            telemetry.addLine("Press UP to increase RPM, DOWN to decrease RPM")
            telemetry.addData("Using bang bang", shooter.bangbang)
            telemetry.update()
        }
    }
}