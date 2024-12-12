package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public class DeadWheelLocalizer implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(DeadWheelLocalizer.class.getSimpleName());
    public final C_DcMotor yLeft;
    public final C_DcMotor yRight;
    public final C_DcMotor x;

    public int yLastLeftET;
    public int yLastRightET;
    public int xLastET;

    public Pose pose = new Pose(DukConstants.HARDWARE.ODOMETER_CENTER);
    public Vector delta = new Vector();

    public DeadWheelLocalizer(HardwareMap hardwareMap) {
        yLeft = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "backLeft"));
        x = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "dummyOdometer"));
        yRight = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "backRight"));

        //Behavior is initialized in DriveTrain
        yLeft.invertRefresh = true;
        yRight.invertRefresh = false;
        x.invertRefresh = false;
        yLeft.toRefresh[5] = true;
        yRight.toRefresh[5] = true;
        x.toRefresh[5] = true;
        yLeft.refreshCache();
        yRight.refreshCache();
        x.refreshCache();
        yLastLeftET = yLeft.getCurrentPosition();
        yLastRightET = yRight.getCurrentPosition();
        xLastET = x.getCurrentPosition();
    }

    @Override
    public void dispatchAllCaches() {
        yLeft.dispatchCache();
        yRight.dispatchCache();
        x.dispatchCache();
    }

    @Override
    public void refreshAllCaches() {
        yLastLeftET = yLeft.getCurrentPosition();
        yLastRightET = yRight.getCurrentPosition();
        xLastET = x.getCurrentPosition();
        yLeft.refreshCache();
        yRight.refreshCache();
        x.refreshCache();

        updateHeadingDelta();
        pose.setH(pose.w + pose.getH());
        updatePositionDelta();
        pose.pos.add(delta);
        pose.vel = new Vector(delta);
        pose.vel.scale(1 / TimeManager.getDeltaTime());
        pose.w /= TimeManager.getDeltaTime();
    }

    @Override
    public void pushTelemetry() {
        loggingBuffer.push("Heading", pose.getH());
        loggingBuffer.push("Heading Delta", pose.w);
        loggingBuffer.push("Position X", pose.pos.getX());
        loggingBuffer.push("Position X Delta", delta.getX());
        loggingBuffer.push("Position Y", pose.pos.getY());
        loggingBuffer.push("Position Y Delta", delta.getY());
        loggingBuffer.push("yLeft Raw", yLeft.getCurrentPosition());
        loggingBuffer.push("x Raw", x.getCurrentPosition());
        loggingBuffer.push("yRight Raw", yRight.getCurrentPosition());
        loggingBuffer.dispatch();

        Vector fieldPos = DukUtilities.ETToFieldCoords(pose);
        DashboardInterface.bufferPacket.fieldOverlay().setFill(DukConstants.DEBUG.STROKES.ODOMETER_WHEELS_STROKE).fillCircle(fieldPos.getX(), fieldPos.getY(), 1);
    }

    @Override
    public void allowDispatch(boolean state) {
        yLeft.allowDispatch(state);
        yRight.allowDispatch(state);
        x.allowDispatch(state);
    }

    private void updateHeadingDelta() {
        pose.w = (yLeft.getCurrentPosition() - yLastLeftET - yRight.getCurrentPosition() + yLastRightET)
            * (Math.PI / DukConstants.HARDWARE.ET_PER_ROBOT_REVOLUTION);
    }

    private void updatePositionDelta() {
        double yTicks = (yLeft.getCurrentPosition() - yLastLeftET + yRight.getCurrentPosition() - yLastRightET) * 0.5;
        double xTicks = x.getCurrentPosition() - xLastET;
//        double averageHeading = pose.getH() - pose.w * 0.5;
//        double hSin = Math.sin(averageHeading);
//        double hCos = Math.cos(averageHeading);
//        delta = new Vector(hSin * yTicks + hCos * xTicks, hCos * yTicks - hSin * xTicks);
//        totalDeltaET = (yTicks + xTicks);
        if (pose.w == 0) {
            double midPoint = pose.getH() - 0.5 * pose.w;
            double sin = Math.sin(midPoint);
            double cos = Math.cos(midPoint);
            delta = new Vector(sin * yTicks + cos * xTicks, cos * yTicks - sin * xTicks);
        } else {
            double sinCurrent = Math.sin(pose.getH());
            double cosCurrent = Math.cos(pose.getH());
            double sinLast = Math.sin(pose.getH() - pose.w);
            double cosLast = Math.cos(pose.getH() - pose.w);
            Vector xDelta = new Vector(sinCurrent - sinLast, cosCurrent - cosLast);
            xDelta.scale(xTicks / pose.w);
            Vector yDelta = new Vector(cosLast - cosCurrent, sinCurrent - sinLast);
            yDelta.scale(yTicks / pose.w);
            xDelta.add(yDelta);
            delta = xDelta;
        }
    }
}
