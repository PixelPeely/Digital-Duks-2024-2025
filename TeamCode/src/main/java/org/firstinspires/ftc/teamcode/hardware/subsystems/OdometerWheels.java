package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;

public class OdometerWheels implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(OdometerWheels.class.getSimpleName());
    public final C_DcMotor yLeft;
    public final C_DcMotor yRight;
    public final C_DcMotor x;

    public int yLastLeftET;
    public int yLastRightET;
    public int xLastET;
    public float totalDeltaET = 0;

    private float heading;
    public float headingOffset;
    public float headingDelta;
    public float positionX;
    public float positionY;
    public float positionDeltaX;
    public float positionDeltaY;

    public OdometerWheels(HardwareMap hardwareMap) {
        yLeft = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "frontLeft"));
        x = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "frontRight"));
        yRight = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "backLeft"));

        //Behavior is initialized in DriveTrain
        yLeft.invertRefresh = true;
        yLeft.toRefresh[5] = true;
        yRight.invertRefresh = false;
        yRight.toRefresh[5] = true;
        x.invertRefresh = true;
        x.toRefresh[5] = true;
        yLeft.refreshCache();
        yRight.refreshCache();
        x.refreshCache();
        yLastLeftET = yLeft.getCurrentPosition();
        yLastRightET = yRight.getCurrentPosition();
        xLastET = x.getCurrentPosition();
        dispatchAllCaches();
    }

    public void setBeginningPose(float x, float y, float h, boolean trueBeginning) {
        positionX = x;
        positionY = y;
        heading = h;
        if (trueBeginning)
            headingOffset = h;
    }

    //Absolute is only used during autonomous and persistent data
    public float getHeading(boolean absolute) {
        return DukUtilities.constrainAxis(heading - (absolute ? 0 : headingOffset));
    }

    public void setRelativeHeading(float _heading) {
        heading = _heading;
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
        heading = DukUtilities.constrainAxis(headingDelta + heading);
        updatePositionDelta();
        positionX += positionDeltaX;
        positionY += positionDeltaY;
    }

    @Override
    public void pushTelemetry() {
        if (!DukConstants.DEBUG.USE_FTC_DASHBOARD) return;

        loggingBuffer.push("Heading", getHeading(false));
        loggingBuffer.push("Absolute Heading", getHeading(true));
        loggingBuffer.push("Heading Delta", headingDelta);
        loggingBuffer.push("Position X", positionX);
        loggingBuffer.push("Position X Delta", positionDeltaX);
        loggingBuffer.push("Position Y", positionY);
        loggingBuffer.push("Position Y Delta", positionDeltaY);
        loggingBuffer.push("yLeft Raw", yLeft.getCurrentPosition());
        loggingBuffer.push("x Raw", x.getCurrentPosition());
        loggingBuffer.push("yRight Raw", yRight.getCurrentPosition());
        loggingBuffer.dispatch();
        DashboardInterface.renderRobot(DukConstants.DEBUG.ROBOT_POSE_STROKE, positionX, positionY, heading);
    }

    @Override
    public void allowDispatch(boolean state) {
        yLeft.allowDispatch(state);
        yRight.allowDispatch(state);
        x.allowDispatch(state);
    }

    private void updateHeadingDelta() {
        headingDelta = (yLeft.getCurrentPosition() - yLastLeftET - yRight.getCurrentPosition() + yLastRightET)
            * (float)(Math.PI / DukConstants.HARDWARE.ET_PER_ROBOT_REVOLUTION_Y);
    }

    private void updatePositionDelta() {
        double yTicks = (yLeft.getCurrentPosition() - yLastLeftET + yRight.getCurrentPosition() - yLastRightET) * 0.5;
        double xTicks = x.getCurrentPosition() - xLastET - headingDelta * (DukConstants.HARDWARE.ET_PER_ROBOT_REVOLUTION_X / (2 * Math.PI));
        double averageHeadingDelta = heading - headingDelta * 0.5;
        double odometerPivotPositionDeltaY = Math.cos(averageHeadingDelta) * yTicks + Math.sin(-averageHeadingDelta) * xTicks;
        double odometerPivotPositionDeltaX = Math.sin(averageHeadingDelta) * yTicks + Math.cos(-averageHeadingDelta) * xTicks;
        float oxCenterPivotTheta = DukUtilities.constrainAxis((float)(heading + DukConstants.HARDWARE.OX_PIVOT_CENTER_THETA));
        positionDeltaY = (float) (odometerPivotPositionDeltaY - Math.cos(oxCenterPivotTheta) * DukConstants.HARDWARE.OX_PIVOT_DIST_FROM_CENTER);
        positionDeltaX = (float) (odometerPivotPositionDeltaX - Math.sin(oxCenterPivotTheta) * DukConstants.HARDWARE.OX_PIVOT_DIST_FROM_CENTER);
        totalDeltaET = (float) (yTicks + xTicks);
    }
}
