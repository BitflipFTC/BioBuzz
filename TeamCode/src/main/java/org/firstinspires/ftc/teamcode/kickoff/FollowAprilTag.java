package org.firstinspires.ftc.teamcode.kickoff;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.SquidController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import club.bitflip.utils.TelemetryImplUpstreamSubmission;
import club.bitflip.utils.hardware.MotorEx;

@Configurable
@TeleOp
public class FollowAprilTag extends LinearOpMode {
    static double kP = 0.013;
    static double kD = 0.00;
    static double kS = 0.055;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), new TelemetryImplUpstreamSubmission(this));

        int[] portalIDS = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        SquidController controller = new SquidController(kP, 0.0, kD, 0.0, kS);

        boolean usingC270 = false;
        boolean lastUsingc270 = false;

        OV9281.viewContainerId = portalIDS[0];
        C270.viewContainerId = portalIDS[1];

        C270 c270 = new C270(hardwareMap, telemetry, false);
        OV9281 ov9281 = new OV9281(hardwareMap, telemetry);

        MotorEx frontleft = new MotorEx("frontleft").zeroed().brake().reverse();
        MotorEx frontright = new MotorEx("frontright").zeroed().brake();
        MotorEx backleft = new MotorEx("backleft").zeroed().brake().reverse();
        MotorEx backright = new MotorEx("backright").zeroed().brake();

        waitForStart();

        AprilTagDetection detection;
        double bearing;

        Pose2D robotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        Drawing.init();

        while (opModeIsActive()) {
            lastUsingc270 = usingC270;

            if (gamepad1.b) usingC270 = !usingC270;

            if (usingC270 && !lastUsingc270) {
                ov9281.disableProcessor();
                c270.enableProcessor();
            } else if (!usingC270 && lastUsingc270) {
                c270.disableProcessor();
                ov9281.enableProcessor();
            }

            if (!usingC270) {
                ov9281.periodic();

                if (ov9281.getDetectionsAmount() > 0) {
                    detection = ov9281.getDetections().get(0);
                    bearing = detection.ftcPose.bearing;
                } else {
                    bearing = 0f;
                }

                robotPose = ov9281.getRobotPose2d();
                Drawing.drawRobot(robotPose);
            } else {
                c270.updateAtag();

                if (c270.getDetectionsAmount() > 0) {
                    detection = c270.getDetections().get(0);
                    bearing = detection.ftcPose.bearing;
                } else {
                    bearing = 0f;
                }

                robotPose = c270.getRobotPose2d();
                Drawing.drawRobot(robotPose);
            }

            controller.setCoeffs(kP, 0, kD, 0, kS);

            double pv = bearing;
            double pow = controller.calculate(pv, 0f);

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

            telemetry.addData("Pow", pow);
            telemetry.addData("Pv", pv);
            telemetry.addData("sp", 0);

            telemetry.update();
        }
    }
}
