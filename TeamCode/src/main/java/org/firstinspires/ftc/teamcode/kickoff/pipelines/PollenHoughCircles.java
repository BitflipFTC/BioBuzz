/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.kickoff.pipelines;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import org.opencv.imgproc.Imgproc;

public class PollenHoughCircles implements VisionProcessor {
    public static class Pollen {
        public final double x;
        public final double y;
        public final double r;

        public Pollen (double x, double y, double r) {
            this.x = x;
            this.y = y;
            this.r = r;
        }
    }

    /*
     * Our working image buffers
     */
    Mat cbMat = new Mat();
    Mat outputImg = new Mat();

//    private Telemetry telemetry = null;

//    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_OPEN, new Size(6, 6));

    private volatile List<Pollen> pollenList = List.of();

    public List<Pollen> getPollenList() {
        return pollenList;
    }

    /*
     * Colors
     */
    static final Scalar TEAL = new Scalar(3, 148, 252);
    static final Scalar PURPLE = new Scalar(158, 52, 235);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);

    static final int CONTOUR_LINE_THICKNESS = 2;
    static final int CB_CHAN_IDX = 2;

    public Scalar params = new Scalar(4.0, 25, 60, 0.8);

    public PollenHoughCircles() {
//        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos)
    {
        input.copyTo(outputImg);

        findCircles(outputImg);

        return outputImg;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.MAGENTA);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(4f);

        for (Pollen p : pollenList) {
            float x = (float) p.x * scaleBmpPxToCanvasPx;
            float y = (float) p.y * scaleBmpPxToCanvasPx;
            float r = (float) p.r * scaleBmpPxToCanvasPx;
            canvas.drawCircle(x,y,r,paint);
        }
    }

    void findCircles(Mat input)
    {
        // Convert the input image to YCrCb color space, then extract the Cb channel
        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);

        Imgproc.medianBlur(cbMat, cbMat, 5);
        Mat circles = new Mat();

        Imgproc.HoughCircles(cbMat, circles, Imgproc.HOUGH_GRADIENT_ALT, params.val[0], params.val[1], params.val[2], params.val[3], 20);

        List<Pollen> bufList = new ArrayList<>();
        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            Point center = new Point(Math.round(c[0]), Math.round(c[1]));
            // circle center
            Imgproc.circle(outputImg, center, 1, new Scalar(0,100,100), (int) ((2/640.0)*outputImg.cols()), 8, 0 );
            // circle outline
            int radius = (int) Math.round(c[2]);
            Imgproc.circle(outputImg, center, radius, new Scalar(255,0,255), 2, 8, 0 );

            bufList.add(new Pollen(center.x, center.y, radius));
        }

        pollenList = List.copyOf(bufList);

        circles.release();
    }
}
