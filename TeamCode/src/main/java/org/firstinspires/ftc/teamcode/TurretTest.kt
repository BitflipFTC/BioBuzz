package org.firstinspires.ftc.teamcode

import club.bitflip.utils.TelemetryImplUpstreamSubmission
import club.bitflip.utils.hardware.ServoEx
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.Utility

@Utility
class TurretTest: LinearOpMode() {
    lateinit var turret1: ServoEx
    lateinit var turret2: ServoEx

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))
        turret1 = ServoEx("turret1")
        turret2 = ServoEx("turret2")

        telemetry.run {
            addData("Initialized", true)
            update()
        }

        turretTo(0.5)
        waitForStart()

        while (opModeIsActive()) {
            turretTo (1 - ((gamepad1.right_stick_x / 2.0) + 0.5))

            telemetry.run {
                addData("servo pos!", gamepad1.right_stick_x / 2 + 0.5)
                update()
            }
        }
    }

    fun turretTo(target: Double) {
        turret1.position = target
        turret2.position = target
    }
}