package org.firstinspires.ftc.teamcode.kickoff;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.kickoff.pipelines.PollenHoughCircles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
public class C270 {
    private PollenHoughCircles processor = null;
    private AprilTagProcessor atag = null;
    private final ArrayList<AprilTagDetection> detectionsBuffer = new ArrayList<>();
    private final VisionPortal visionPortal;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private long defaultExposure;
    private int defaultGain;
    public static int viewContainerId = -1;

    public final int resolutionWidth = 320;
    public final int resolutionHeight = 240;

    private final HardwareMap hwMap;
    private final Telemetry tele;

    private List<PollenHoughCircles.Pollen> pollenList = List.of();

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public List<PollenHoughCircles.Pollen> getPollenList() {
        return pollenList;
    }

    public C270 (HardwareMap hwMap, Telemetry tele, boolean houghCircles) {
        this.hwMap = hwMap;
        this.tele = tele;

        if (houghCircles) {
            processor = new PollenHoughCircles();

            if (viewContainerId == -1) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hwMap.get(WebcamName.class, "c"))
                        .setCameraResolution(new Size(resolutionWidth, resolutionHeight))
                        .setShowStatsOverlay(true)
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .addProcessor(processor)
                        .setAutoStopLiveView(true)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hwMap.get(WebcamName.class, "c"))
                        .setCameraResolution(new Size(resolutionWidth, resolutionHeight))
                        .setShowStatsOverlay(true)
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .setLiveViewContainerId(viewContainerId)
                        .addProcessor(processor)
                        .build();
            }
        } else {
            atag = new AprilTagProcessor.Builder()
                    .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                    .setDrawTagOutline(true)
                    .setDrawTagID(true)
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setNumThreads(3)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build();

            if (viewContainerId == -1) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hwMap.get(WebcamName.class, "c"))
                        .setCameraResolution(new Size(640,480))
                        .setShowStatsOverlay(true)
                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                        .addProcessor(atag)
                        .setAutoStopLiveView(true)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hwMap.get(WebcamName.class, "c"))
                        .setCameraResolution(new Size(640,480))
                        .setShowStatsOverlay(true)
                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                        .setLiveViewContainerId(viewContainerId)
                        .addProcessor(atag)
                        .build();
            }
        }

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        gainControl = visionPortal.getCameraControl(GainControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        defaultExposure = exposureControl.getExposure(TimeUnit.MILLISECONDS);
        defaultGain = gainControl.getGain();
        exposureControl.setExposure(15, TimeUnit.MILLISECONDS);
        gainControl.setGain(15);
    }

    public void resetExposureGain () {
        if (exposureControl != null && gainControl != null) {
            exposureControl.setMode(ExposureControl.Mode.Auto);
            exposureControl.setExposure(defaultExposure,TimeUnit.MILLISECONDS);
            gainControl.setGain(defaultGain);
        }
    }

    public void setExposure (int exposure) {
        exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
    }

    public void disableProcessor() {
        visionPortal.setProcessorEnabled(processor, false);
    }

    public void enableProcessor() {
        visionPortal.setProcessorEnabled(processor, true);
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public void updatePollenList() {
        pollenList = processor.getPollenList();
    }

    public void updateAtag() {
        detectionsBuffer.clear();
        if (atag.getDetections() != null) {
            detectionsBuffer.addAll(atag.getDetections());
        }

        int count = detectionsBuffer.size();

        if (count == 0) {
            if (true)
                tele.addData("Detected April Tags", 0);
            return;
        }

        if (true)
            tele.addData("Detected April Tags", detectionsBuffer.size());
        for (AprilTagDetection detection : detectionsBuffer) {
            if (true) {
                tele.addData("ID", detection.id);
                tele.addData("Sureness", detection.decisionMargin);
            }
        }
    }

    public int getDetectionsAmount () { return detectionsBuffer.size(); }

    public ArrayList<AprilTagDetection> getDetections() {
        return detectionsBuffer;
    }
}
