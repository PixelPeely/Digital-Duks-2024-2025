package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public class PoseEstimator implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(PoseEstimator.class.getSimpleName());
    public final OdometerWheels odometerWheels;
    public final DuplexIMU duplexIMU;

    public static class Pose {

        public Vector pos;
        public Vector vel;
        private double h;
        public double w;


        public Pose() {
            pos = new Vector();
            vel = new Vector();
        }

        public Pose(Vector _pos) {
            pos = new Vector(_pos);
            vel = new Vector();
        }

        public Pose(Vector _pos, double _h) {
            pos = new Vector(_pos);
            vel = new Vector();
            setH(_h);
        }

        public Pose(Vector _pos, Vector _vel, double _h, double _w) {
            pos = new Vector(_pos);
            vel = _vel;
            setH(_h);
            w = _w;
        }

        public Pose(Pose pose) {
            pos = new Vector(pose.pos);
            vel = new Vector(pose.vel);
            h = pose.h;
            w = pose.w;
        }

        public double getH() {return h;}

        public void setH(double _h) {
            h = DukUtilities.angleWrap(_h);
        }

        /**
         * Transform this pose relative to its parent's space
         * @param heading Heading of the parent
         * @param offset Location of this pose relative to the parent's center
         * @param toGlobal Weather to transform to parent's global space or reverse the effect
         */
        public void spaceTransform(double heading, Vector offset, boolean toGlobal) {
            Vector _offset = new Vector(offset);
            if (toGlobal) _offset.negate();//Swap with line below if issues arise
            _offset.rotate(heading);
            pos.add(_offset);
        }

    }

    private Pose pose = new Pose();

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
        Pose wheels = new Pose(_pose);
        wheels.spaceTransform(pose.getH(), DukConstants.HARDWARE.ODOMETER_CENTER, false);
        //odometerWheels.pose = wheels;
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

        Pose wheels = new Pose(odometerWheels.pose);
        wheels.spaceTransform(odometerWheels.pose.getH(), DukConstants.HARDWARE.ODOMETER_CENTER, true);

        setPose(new Pose(wheels.pos, wheels.vel, wheels.getH(), wheels.w / TimeManager.getDeltaTime()));
    }

    @Override
    public void pushTelemetry() {
        loggingBuffer.push("Pose X", pose.pos.getX());
        loggingBuffer.push("Pose Y", pose.pos.getY());
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
