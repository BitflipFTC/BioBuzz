package org.firstinspires.ftc.teamcode.kickoff;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SquidController;
import org.firstinspires.ftc.teamcode.kickoff.pipelines.PollenHoughCircles;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;

import club.bitflip.utils.PIDController;
import club.bitflip.utils.hardware.MotorEx;

@Configurable
@TeleOp
public class KickoffCVPresentation extends LinearOpMode {
    static double kP = 0.013;
    static double kD = 0.00;
    static double kS = 0.055;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

//        int[] portalIDS = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        SquidController controller = new SquidController(kP, 0.0, kD, 0.0, kS);

//        OV9281.viewContainerId = portalIDS[0];
//        C270.viewContainerId = portalIDS[1];

//        OV9281 ov9281 = new OV9281(hardwareMap, telemetry);
        C270 c270 = new C270(hardwareMap, telemetry);

        MotorEx frontleft = new MotorEx("frontleft").zeroed().brake().reverse();
        MotorEx frontright = new MotorEx("frontright").zeroed().brake();
        MotorEx backleft = new MotorEx("backleft").zeroed().brake().reverse();
        MotorEx backright = new MotorEx("backright").zeroed().brake();

        ArrayList<PollenHoughCircles.Pollen> pollenList;
        PollenHoughCircles.Pollen biggest = new PollenHoughCircles.Pollen(160,120,1);

        waitForStart();

        while (opModeIsActive()) {
//            ov9281.periodic();
            c270.periodic();

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
            frontleft.setPower(pow);
            backleft.setPower(pow);
            frontright.setPower(-pow);
            backright.setPower(-pow);

            telemetry.update();
        }
    }
}
