package org.firstinspires.ftc.teamcode

import club.bitflip.utils.TelemetryImplUpstreamSubmission
import club.bitflip.utils.hardware.MotorEx
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs
import kotlin.math.max

@TeleOp
class DefaultDrive: LinearOpMode() {
    enum class Speed (val power: Double) {
        MIN(0.25),
        MED(0.5),
        HIGH(0.75),
        MAX(1.0)
    }

    val speedArray = Speed.entries.toTypedArray()

    private var currentSpeed: Speed = Speed.MED

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))
        val frontleft = MotorEx("frontleft").zeroed().brake().reverse()
        val frontright = MotorEx("frontright").zeroed().brake()
        val backleft = MotorEx("backleft").zeroed().brake().reverse()
        val backright = MotorEx("backright").zeroed().brake()

        telemetry.run {
            addData("Initialized", true)
            update()
        }

        waitForStart()

        while (opModeIsActive()) {
            if (gamepad1.rightBumperWasPressed()) {
                currentSpeed = speedArray[currentSpeed.ordinal + 1]
            }

            if (gamepad1.leftBumperWasPressed()) {
                currentSpeed = speedArray[currentSpeed.ordinal - 1]
            }

            val x = gamepad1.left_stick_x.toDouble()
            val y = -gamepad1.left_stick_y.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()

            val max: Double
            var flPower: Double = (y + x + rx) * currentSpeed.power
            var frPower: Double = (y - x - rx) * currentSpeed.power
            var blPower: Double = (y - x + rx) * currentSpeed.power
            var brPower: Double = (y + x - rx) * currentSpeed.power

            max = max(abs(max(max(abs(frPower), abs(flPower)), abs(blPower))), abs(brPower))

            telemetry.addData("Current speed setting", currentSpeed.name)
            telemetry.addData("Current speed value", currentSpeed.power)

            if (max > 1) {
                flPower /= max
                frPower /= max
                blPower /= max
                brPower /= max
            }

            frontleft.power = flPower
            frontright.power = frPower
            backleft.power = blPower
            backright.power = brPower

            if (gamepad1.a) backleft.power = currentSpeed.power
            if (gamepad1.x) frontleft.power = currentSpeed.power
            if (gamepad1.y) frontright.power = currentSpeed.power
            if (gamepad1.b) backright.power = currentSpeed.power

            telemetry.addLine("---------------------------------------")
            telemetry.addData("A", "back_left")
            telemetry.addData("X", "front_left")
            telemetry.addData("Y", "front_right")
            telemetry.addData("B", "back_right")

            telemetry.update()
        }
    }
}