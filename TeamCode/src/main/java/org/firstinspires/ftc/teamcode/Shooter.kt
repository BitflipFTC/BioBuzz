package org.firstinspires.ftc.teamcode

import club.bitflip.utils.TelemetryImplUpstreamSubmission
import club.bitflip.utils.hardware.MotorEx
import club.bitflip.utils.hardware.ServoEx
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx

@TeleOp(name = "Tuning shooter")
class Shooter : LinearOpMode() {
    override fun runOpMode() {

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))

        var output = 0.0
        var tbh = 0.0
        val gain = 0.001

        val flywheelMotor = MotorEx("shooter")
        val servo = ServoEx("servo")
        var flywheelRPM = 0.0
        var targetFlywheelRPM = 0.0
        var error = 0.0
        var lastError = 0.0

        var servoPollenPos = 0.5
        var servoNectarPos = 1.0
        var servoPos = servoNectarPos

        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                targetFlywheelRPM += 125.0
            }

            if (gamepad1.dpadDownWasPressed()) {
                targetFlywheelRPM -= 125.0
            }

            if (gamepad1.dpadLeftWasPressed()) {
                servoPos = servoPollenPos
            }

            if (gamepad1.dpadRightWasPressed()) {
                servoPos = servoNectarPos
            }

            if (gamepad1.rightBumperWasPressed()) {
                servoPos += 0.025
            }

            if (gamepad1.leftBumperWasPressed()) {
                servoPos -= 0.025
            }

            if (gamepad1.aWasPressed()) {
                servoPollenPos = servoPos
            }

            if (gamepad1.bWasPressed()) {
                servoNectarPos = servoPos
            }

            flywheelRPM = (flywheelMotor.velocity / 28) * 60

            lastError = error
            error = targetFlywheelRPM - flywheelRPM

            if (error >= 40.0) {
                flywheelMotor.power = 1.0
                output = targetFlywheelRPM * (1.0/5000.0)
                tbh = output
            } else {
                output += gain * error
                output = output.coerceIn(-0.05,1.0)

                if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
                    output = 0.5 * (output + tbh)
                    tbh = output
                }

                output = output.coerceIn(0.01,1.0)

                flywheelMotor.power = output
            }

            servoPos = servoPos.coerceIn(0.0,1.0)
            servo.position = servoPos




            telemetry.addData("Flywheel RPM", flywheelRPM)
            telemetry.addData("Target Flywheel RPM", targetFlywheelRPM)
            telemetry.addData("Output", output)
            telemetry.addData("Servo Position", servoPos)
            telemetry.addLine()
            telemetry.addLine("Press UP to increase RPM, DOWN to decrease RPM")
            telemetry.addLine("Press RIGHT to switch to nectar, LEFT to switch to pollen")
            telemetry.addLine("Press LB to decrease servo position, RB to increase servo position")
            telemetry.addLine("Press A to assign pollen position, B to assign nectar position")
            telemetry.update()
        }
    }
}
