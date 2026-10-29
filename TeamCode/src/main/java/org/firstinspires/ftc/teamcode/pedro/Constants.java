package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
//        return null;
    }

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontleft");
                c.backLeftName.set("backleft");
                c.frontRightName.set("frontright");
                c.backRightName.set("backright");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.625); //-4.34646
        c.yPodOffset.set(-4.1); //-3.48064
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2976163721793443);
                Controller secondaryTranslationalForward = Controller.proportional(0.10996129706434059);
                Controller primaryTranslationalLateral = Controller.proportional(0.435696956128987);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16097838325257843);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01394644525101465));
                c.brake.set(Controller.proportionalFeedforward(0.011854478463362452));

                c.headingFeedback.set(Controller.proportional(4.902852101852481));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05255258794818154, 0.004570749497054143));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09437754540518345, 0.10693945454454636));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011591993749757263, 2.810967940080824E-4));

                c.maxAchievableForwardVelocity.set(73.43872734899817);
                c.maxAchievableStrafeVelocity.set(61.226875431706205);
                c.naturalForwardDeceleration.set(41.23408279011008);
                c.naturalStrafeDeceleration.set(76.20652169379306);
                c.maxPathSpeed.set(0.7);
            }
    );
}