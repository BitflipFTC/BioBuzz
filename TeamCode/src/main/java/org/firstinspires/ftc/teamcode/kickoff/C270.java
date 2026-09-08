package org.firstinspires.ftc.teamcode.kickoff;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.kickoff.pipelines.PollenHoughCircles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
public class C270 {

    private final PollenHoughCircles processor;
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

    // exposure: 1-7
    // gain: 1-6
    public C270 (HardwareMap hwMap, Telemetry tele) {
        this.hwMap = hwMap;
        this.tele = tele;

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

    public void periodic() {
        pollenList = processor.getPollenList();
    }
}
