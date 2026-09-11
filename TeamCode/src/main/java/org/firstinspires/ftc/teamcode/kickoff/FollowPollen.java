package org.firstinspires.ftc.teamcode.kickoff;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SquidController;
import org.firstinspires.ftc.teamcode.kickoff.pipelines.PollenHoughCircles;

import java.util.ArrayList;

import club.bitflip.utils.TelemetryImplUpstreamSubmission;
import club.bitflip.utils.hardware.MotorEx;

@Configurable
@TeleOp
public class FollowPollen extends LinearOpMode {
    static double kP = 0.013;
    static double kD = 0.00;
    static double kS = 0.055;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), new TelemetryImplUpstreamSubmission(this));

        SquidController controller = new SquidController(kP, 0.0, kD, 0.0, kS);

        C270 c270 = new C270(hardwareMap, telemetry, true);

        MotorEx frontleft = new MotorEx("frontleft").zeroed().brake().reverse();
        MotorEx frontright = new MotorEx("frontright").zeroed().brake();
        MotorEx backleft = new MotorEx("backleft").zeroed().brake().reverse();
        MotorEx backright = new MotorEx("backright").zeroed().brake();

        ArrayList<PollenHoughCircles.Pollen> pollenList;
        PollenHoughCircles.Pollen biggest = new PollenHoughCircles.Pollen(160,120,1);

        waitForStart();

        while (opModeIsActive()) {
            c270.updatePollenList();

            pollenList = new ArrayList<>(c270.getPollenList());

            pollenList.sort((a,b) -> Double.compare(b.r, a.r));

            if (!pollenList.isEmpty()) {
                biggest = pollenList.get(0);
            } else {
                biggest = new PollenHoughCircles.Pollen(159,120,1);
            }

            telemetry.addData("Biggest pollen", "x=%.1f y=%.1f r=%.1f", biggest.x, biggest.y, biggest.r);

            controller.setCoeffs(kP, 0, kD, 0, kS);
            double pow = controller.calculate(-biggest.x + (double) c270.resolutionWidth/2f, 0f);

            telemetry.addData("Pow", pow);
            telemetry.addData("Pv", -biggest.x + (double) c270.resolutionWidth / 2f);
            telemetry.addData("sp", 0);

            if (gamepad1.right_trigger >= 0.1) {
                frontleft.setPower(pow);
                backleft.setPower(pow);
                frontright.setPower(-pow);
                backright.setPower(-pow);
            } else {
                frontleft.setPower(0);
                backleft.setPower(0);
                frontright.setPower(0);
                backright.setPower(0);
            }

            telemetry.update();
        }
    }
}
