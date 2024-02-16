package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.provider.ContactsContract;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.ObservedFieldObject;
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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;

public class DukEye {
    private final OpenCvWebcam webcam;

    public int spikeIndex = 0;
    private static boolean canWriteMemory = true;
    private static HashMap<ObservedFieldObject.Type, List<ObservedFieldObject>> observedFieldObjects = new HashMap<>();

    public DukEye(HardwareMap hardwareMap) {
        for (ObservedFieldObject.Type object : ObservedFieldObject.Type.values())
            observedFieldObjects.put(object, new ArrayList<>());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.tryGet(WebcamName.class, "mainCam"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        setCameraPipeline(0);
        webcam.setMillisecondsPermissionTimeout(5000);
//        int viewPortID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        WebcamName webcamName = hardwareMap.tryGet(WebcamName.class, "mainCam");
////        if (webcamName == null) {
////            webcam = null;
////            return;
////        };
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, viewPortID);
//
//        webcam.setPipeline(new ObjectTriangulation());
//        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                setCameraStreamState(true);
            }

            @Override
            public void onError(int errorCode) {
                if (DukConstants.DEBUG.USE_FTC_DASHBOARD)
                    DashboardInterface.logError("Error setting up camera", errorCode);
            }
        });
    }

    public void setCameraStreamState(boolean enabled) {
        canWriteMemory = enabled;
        if (enabled) {
            webcam.startStreaming(DukConstants.WEBCAM.RESOLUTION_WIDTH, DukConstants.WEBCAM.RESOLUTION_HEIGHT, DukConstants.WEBCAM.ORIENTATION);
            if (DukConstants.DEBUG.USE_FTC_DASHBOARD)
                DashboardInterface.dashboard.startCameraStream(webcam, 0);
        } else {
            webcam.stopStreaming();
            if (DukConstants.DEBUG.USE_FTC_DASHBOARD)
                DashboardInterface.dashboard.stopCameraStream();
        }
    }

    public ObservedFieldObject getLargestObject(ObservedFieldObject.Type object) {
        canWriteMemory = false;
        ObservedFieldObject largestObject = new ObservedFieldObject(0, 0, new Rect());
        for (ObservedFieldObject fieldObject : observedFieldObjects.get(object))
            if (fieldObject.boundingRect.area() > largestObject.boundingRect.area())
                largestObject = fieldObject;
        canWriteMemory = true;
        return largestObject;
    }

    public void observeSpikeMark(boolean red) {
        ObservedFieldObject prop = getLargestObject(red ? ObservedFieldObject.Type.TEAM_PROP_RED : ObservedFieldObject.Type.TEAM_PROP_BLUE);
        if (prop.xRelative == 0 || prop.yRelative == 0) {
            spikeIndex = 0;
            return;
        }

        float centerX = prop.boundingRect.x + prop.boundingRect.width * 0.5f;
        if (centerX < DukConstants.WEBCAM.SPIKE_LEFT_BOUNDARY)
            spikeIndex = 1;
        else if (centerX < DukConstants.WEBCAM.SPIKE_MIDDLE_BOUNDARY)
            spikeIndex = 2;
        else
            spikeIndex = 3;

        if (DukConstants.DEBUG.USE_FTC_DASHBOARD) {
            DashboardInterface.logError("Spike", spikeIndex);
            DashboardInterface.logError("Prop X", prop.boundingRect.x);
        }
    }

    public void setCameraPipeline(int index) {
        switch (index) {
            case 0:
                webcam.setPipeline(new ObjectTriangulation());
                break;
            case 1:
                webcam.setPipeline(new ObstacleAvoidance());
                break;
        }
    }

    private static class ObjectTriangulation extends OpenCvPipeline {
        //Pre-allocated mat variables
        Mat output = new Mat();
        Mat HSV = new Mat();
        Mat YCbCr = new Mat();
        Mat contourHierarchy = new Mat();
        Mat[] objectMasks = new Mat[ObservedFieldObject.Type.values().length];
        List<MatOfPoint>[] objectContours = new ArrayList[ObservedFieldObject.Type.values().length];
        List<Rect>[] objectBounds = new ArrayList[ObservedFieldObject.Type.values().length];

        int currentImageLayer = 0;
        float yOffset = -DukConstants.WEBCAM.HEIGHT / (float)Math.tan(DukConstants.WEBCAM.PITCH);
        float dOffset = (float)Math.sqrt(yOffset * yOffset + DukConstants.WEBCAM.HEIGHT * DukConstants.WEBCAM.HEIGHT);
        float pitchCos = (float)Math.cos(DukConstants.WEBCAM.PITCH);

        private void initPreallocatedMemory() {
            for (ObservedFieldObject.Type object : ObservedFieldObject.Type.values()) {
                objectMasks[object.ordinal()] = new Mat();
                objectContours[object.ordinal()] = new ArrayList<>();
                objectBounds[object.ordinal()] = new ArrayList<>();
            }
        }
        
        @Override
        public Mat processFrame(Mat input) {
            if (!canWriteMemory) return input;
            if (currentImageLayer == 0) initPreallocatedMemory();
            currentImageLayer = 1;
            input.copyTo(output);
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            for (ObservedFieldObject.Type object : ObservedFieldObject.Type.values()) {
                processObjectMask(object);
                processObjectContours(object);
                triangulateObjects(object);
            }

            postProcessOutput();
            return output;
        }

        private void processObjectMask(ObservedFieldObject.Type object) {
            Scalar[] masks = DukConstants.WEBCAM.COLOR_SCALARS.get(object);
            Core.inRange(masks[0].val[3] == Imgproc.COLOR_RGB2HSV ? HSV : YCbCr, masks[0], masks[1], objectMasks[object.ordinal()]);
            renderImageLayer(objectMasks[object.ordinal()]::copyTo);
        }

        private void processObjectContours(ObservedFieldObject.Type object) {
            objectContours[object.ordinal()].clear();
            objectBounds[object.ordinal()].clear();
            Imgproc.findContours(objectMasks[object.ordinal()], objectContours[object.ordinal()], contourHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            renderImageLayer(o -> Imgproc.drawContours(o, objectContours[object.ordinal()], 5, new Scalar(255, 255, 255)));

            for (MatOfPoint contour : objectContours[object.ordinal()]) {
                Rect rect = Imgproc.boundingRect(contour);
                if (rect.area() > DukConstants.WEBCAM.MIN_BOUNDING_AREA)
                    objectBounds[object.ordinal()].add(rect);
            }
        }

        private float mapCoord(float coord, boolean horizontal) {
            return 0.5f * (horizontal ? DukConstants.WEBCAM.YAW_FOV * (2 * coord / DukConstants.WEBCAM.RESOLUTION_WIDTH - 1)
                    : DukConstants.WEBCAM.PITCH_FOV * (1 - 2 * coord / DukConstants.WEBCAM.RESOLUTION_HEIGHT));
        }

        private void triangulateObjects(ObservedFieldObject.Type object) {
            observedFieldObjects.get(object).clear();
            for (Rect rect : objectBounds[object.ordinal()]) {
                if (rect.x == 0 || rect.x + rect.width > DukConstants.WEBCAM.RESOLUTION_WIDTH
                    || rect.y == 0 || rect.y + rect.height > DukConstants.WEBCAM.RESOLUTION_HEIGHT) break;

                float yTop = -DukConstants.WEBCAM.HEIGHT / (float) Math.tan(DukConstants.WEBCAM.PITCH + mapCoord(rect.y, false));
                float xLeft = (dOffset + (yTop - yOffset) * pitchCos) * (float) Math.tan(mapCoord(rect.x, true));
                float yOvershoot = yTop * DukConstants.HARDWARE.PIXEL_HEIGHT / DukConstants.WEBCAM.HEIGHT;
                float xOvershoot = yOvershoot * xLeft / yTop + xLeft * DukConstants.HARDWARE.PIXEL_HEIGHT / DukConstants.WEBCAM.HEIGHT;
                observedFieldObjects.get(object).add(new ObservedFieldObject(xLeft - xOvershoot, yTop - yOvershoot, rect));
            }
            renderImageLayer(o -> {
                for (ObservedFieldObject fieldObject : observedFieldObjects.get(object)) {
                    Imgproc.rectangle(o, fieldObject.boundingRect, new Scalar(255, 255, 255));
                    Imgproc.putText(o, object.name(), new Point(fieldObject.boundingRect.x, fieldObject.boundingRect.y), 1, 2, new Scalar(255, 255, 255));
                    Imgproc.putText(o, "X:" + DukConstants.WEBCAM.COORD_TEXT_FORMAT.format(fieldObject.xRelative) +
                            ",Y:" + DukConstants.WEBCAM.COORD_TEXT_FORMAT.format(fieldObject.yRelative),
                            new Point(fieldObject.boundingRect.x, fieldObject.boundingRect.y + 50), 1, 2, new Scalar(255, 255, 255));
                }
            });
        }

        private void postProcessOutput() {
            Imgproc.line(output,
                    new Point(DukConstants.WEBCAM.SPIKE_LEFT_BOUNDARY, 0),
                    new Point(DukConstants.WEBCAM.SPIKE_LEFT_BOUNDARY, DukConstants.WEBCAM.RESOLUTION_HEIGHT),
                    new Scalar(255, 255, 255), 2);
            Imgproc.line(output,
                    new Point(DukConstants.WEBCAM.SPIKE_MIDDLE_BOUNDARY, 0),
                    new Point(DukConstants.WEBCAM.SPIKE_MIDDLE_BOUNDARY, DukConstants.WEBCAM.RESOLUTION_HEIGHT),
                    new Scalar(255, 255, 255), 2);
        }

        private void renderImageLayer(Consumer<Mat> process) {
            if (DukConstants.WEBCAM.IMAGE_LAYER == currentImageLayer) process.accept(output);
            currentImageLayer++;
        }
    }

    private static class ObstacleAvoidance extends OpenCvPipeline {


        @Override
        public Mat processFrame(Mat input) {
            return null;
        }
    }
}
