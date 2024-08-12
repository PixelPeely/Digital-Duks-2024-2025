package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public class PoseEstimator implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(PoseEstimator.class.getSimpleName());
    public static class Pose {
        public float x;
        public float y;
        private float h;
        public float w;
        public float s;
        public float vx;
        public float vy;

        public Pose() {}

        public Pose(float _x, float _y) {
            x = _x;
            y = _y;
        }

        public Pose(float _x, float _y, float _h) {
            x = _x;
            y = _y;
            setH(_h);
        }

        public Pose(float _x, float _y, float _h, float _vx, float _vy, float _w) {
            x = _x;
            y = _y;
            setH(_h);
            vx = _vx;
            vy = _vy;
            s = (float)Math.sqrt(vx * vx + vy * vy);
            w = _w;
        }

        public Pose(Pose pose) {
            x = pose.x;
            y = pose.y;
            h = pose.h;
            w = pose.w;
            s = pose.s;
            vx = pose.vx;
            vy = pose.vy;
        }

        public float getH() {return h;}

        public void setH(float _h) {
            h = DukUtilities.constrainAxis(_h);
        }
    }

    private Pose pose = new Pose();

    private final OdometerWheels odometerWheels;
    private final DuplexIMU duplexIMU;

    public PoseEstimator(HardwareMap hMap) {
        odometerWheels = new OdometerWheels(hMap);
        duplexIMU = new DuplexIMU(hMap);

//        TimeManager.hookPeriodic(DukConstants.ORIENTATION.IMU_CHECK_INTERVAL, t -> {
//            pose.setH(duplexIMU.getOrientation().thirdAngle);
//            return false;
//        });
    }
    public void setPose(Pose _pose) {
        pose = new Pose(_pose);
        Pose wheelPose = new Pose(_pose);
        DukUtilities.mapPose(pose.getH(), wheelPose, DukConstants.HARDWARE.ODOMETER_CENTER, false);
        odometerWheels.pose = wheelPose;
    }

    public Pose getPose() {return pose;}

    @Override
    public void dispatchAllCaches() {
        odometerWheels.dispatchAllCaches();
        duplexIMU.dispatchAllCaches();
    }

    @Override
    public void refreshAllCaches() {
        odometerWheels.refreshAllCaches();
        duplexIMU.refreshAllCaches();
        float inverseDelta = 1/(float)TimeManager.getDeltaTime();

        Pose wheels = new Pose(odometerWheels.pose);
        DukUtilities.mapPose(odometerWheels.pose.getH(), wheels, DukConstants.HARDWARE.ODOMETER_CENTER, true);

        setPose(new Pose(wheels.x, wheels.y, wheels.getH(),
                wheels.vx * inverseDelta,
                wheels.vy * inverseDelta,
                wheels.w * inverseDelta));
    }

    @Override
    public void pushTelemetry() {
        loggingBuffer.push("Pose X", pose.x);
        loggingBuffer.push("Pose Y", pose.y);
        loggingBuffer.push("Pose H", pose.getH());
        loggingBuffer.dispatch();
        DashboardInterface.renderRobot(DukConstants.DEBUG.STROKES.ROBOT_POSE_STROKE, pose);

        odometerWheels.pushTelemetry();
        duplexIMU.pushTelemetry();
    }

    @Override
    public void allowDispatch(boolean state) {
        odometerWheels.allowDispatch(state);
        duplexIMU.allowDispatch(state);
    }
}
