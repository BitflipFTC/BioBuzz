package org.firstinspires.ftc.teamcode.testing;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.anygeneric.blazeftc.DummyPlugOpMode;
import dev.anygeneric.blazeftc_pedro.PedroSingleDataLocalizer;

@TeleOp
public class ExamplePedroHighSpeedLocalization extends DummyPlugOpMode {
    @Override
    public void runOpModeInBlaze() {
        Telemetry tele = initializeBlazeFTC(telemetry);
        engageMotorAcceleration();
        Follower follower = Constants.createFollower(hardwareMap);
        waitForStart();
        ElapsedTime elt = new ElapsedTime();
        PedroSingleDataLocalizer.setup(follower, () -> {
            tele.addData("pedro loop time (ms)", elt.milliseconds());
            Log.d("TELE", "loop time(ms): " + elt.milliseconds());
            elt.reset();
            follower.update();
            tele.addData("x,y", follower.getPose().getX() + ", " + follower.getPose().getY());
        });
        follower.followPath(new Path(new BezierLine(new Pose(0, 0), new Pose(10, 0))));
        runBlazeFTC(0);
        while (!isStopRequested()) {
            sleep(20);
            tele.update();
        }
    }
}