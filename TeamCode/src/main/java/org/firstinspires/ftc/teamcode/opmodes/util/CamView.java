package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DukEye;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "UTIL CamView")
public class CamView extends OpMode {
    public static double HU = 255;//150
    public static double HL = 0;//0
    public static double SU = 120;//255
    public static double SL = 0;//100
    public static double VU = 190;//255
    public static double VL = 150;//0
    public static double CAMERA_HEIGHT = 88.7;
    public static double PROJECTION_RAY_THETA = -0.244;
    OpenCvWebcam cam;
    public static double numContours;
    private static final DecimalFormat decimalFormat = new DecimalFormat("#.###");
    DukEye eye;

    @Override
    public void init() {
        DashboardInterface.dashboard = FtcDashboard.getInstance();
        //eye = new DukEye(hardwareMap);
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.tryGet(WebcamName.class, "mainCam"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        cam.setPipeline(new MainPipeline());
        cam.setMillisecondsPermissionTimeout(5000);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                if (DukConstants.DEBUG.USE_FTC_DASHBOARD)
                    FtcDashboard.getInstance().startCameraStream(cam, 0);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error setting up camera", errorCode);
            }
        });
        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    @Override
    public void loop() {
//        DashboardInterface.applyConfig();
//        eye.observeSpikeMark(true);
//        DashboardInterface.bufferPacket.put("Spike Location", eye.spikeIndex);
//        DashboardInterface.dispatchBufferPacket();
    }

    static class MainPipeline extends OpenCvPipeline {
        Mat output = new Mat();
        Mat HSV = new Mat();
        Mat greenPixels = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        MatOfPoint largestContour = new MatOfPoint();
        Mat hierarchy = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(output);
            //contours.clear();
            Scalar lowerGreen = new Scalar(HL, SL, VL, 0);
            Scalar upperGreen = new Scalar(HU, SU, VU);
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(HSV, lowerGreen, upperGreen, greenPixels);
//            Imgproc.findContours(greenPixels, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//            getLargestContour(contours);
//            if (largestContour != null && false) {
//                Rect bounds = Imgproc.boundingRect(largestContour);
//                Imgproc.rectangle(output, bounds, new Scalar(0, 255, 0));
//                double thetaPitch = Math.toRadians(43.3) * (1 - bounds.y / 180f);
//                double thetaYaw = Math.toRadians(70.42) * (bounds.x / 320f - 1);
//                Imgproc.circle(output, new Point(bounds.x, bounds.y), 5, new Scalar(255, 0, 0));
//                Imgproc.putText(output, "Pitch: " + decimalFormat.format(Math.toDegrees(thetaPitch)) + ", Yaw: " + decimalFormat.format(Math.toDegrees(thetaYaw)), new Point(bounds.x, bounds.y + 10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5f, new Scalar(0, 255, 0), 2);
//
//                double yFixed = -CAMERA_HEIGHT / Math.tan(PROJECTION_RAY_THETA);
//                double y = -CAMERA_HEIGHT / Math.tan(PROJECTION_RAY_THETA + thetaPitch);
//                double rayDistance = Math.sqrt(yFixed * yFixed + CAMERA_HEIGHT * CAMERA_HEIGHT)
//                        + (y - yFixed) * Math.cos(PROJECTION_RAY_THETA);
//                double x = rayDistance * Math.tan(thetaYaw);
//                double thetaTrueYaw = Math.atan2(x, CAMERA_HEIGHT);
//                double trueY = y - 1.27f / Math.tan(thetaTrueYaw);
//                double trueX = x - 1.27f / Math.tan(PROJECTION_RAY_THETA + thetaPitch);
//
//                Imgproc.putText(output, "Y: " + decimalFormat.format(y) + ", X: " + decimalFormat.format(x), new Point(bounds.x, bounds.y - 10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5f, new Scalar(0, 255, 0), 2);
//                Imgproc.putText(output, "TY: " + decimalFormat.format(trueY) + ", TX: " + decimalFormat.format(trueX), new Point(bounds.x, bounds.y - 30), Imgproc.FONT_HERSHEY_COMPLEX, 0.5f, new Scalar(0, 255, 0), 2);
//            }
//            Imgproc.line(output, new Point(320, 0), new Point(320, 360), new Scalar(255, 255, 255));
//            //Imgproc.drawContours(output, contours, contours.indexOf(largestContour), new Scalar(255, 255, 255), 2);
//            numContours = contours.size();
            return greenPixels;
        }

        public void getLargestContour(List<MatOfPoint> contours) {
            double largestArea = 0;
            largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > largestArea) {
                    largestArea = area;
                    largestContour = contour;
                }
            }
        }
    }
}
