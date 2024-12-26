package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_IMU;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;

public class DuplexIMU implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(DuplexIMU.class.getSimpleName());
    public final C_IMU imuHub;
    public final C_IMU imuExp;

    private Orientation lastOrientation = DukConstants.ORIENTATION.NULL_ORIENTATION;

    public DuplexIMU(HardwareMap hardwareMap) {
        imuHub = new C_IMU(hardwareMap.tryGet(IMU.class, "imuHub"));
        imuExp = new C_IMU(hardwareMap.tryGet(IMU.class, "imuExp"));
        imuHub.initialize(new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imuExp.initialize(new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        imuHub.axesReference = imuExp.axesReference = DukConstants.ORIENTATION.AXES_REFERENCE;
        imuHub.axesOrder = imuExp.axesOrder = DukConstants.ORIENTATION.AXES_ORDER;
        imuHub.angleUnit = imuExp.angleUnit = DukConstants.ORIENTATION.ANGLE_UNIT;

        imuHub.toRefresh[0] = true;
        imuExp.toRefresh[0] = true;

        imuHub.invertRefresh = true;
        imuExp.invertRefresh = true;

        dispatchAllCaches();
    }

    public Orientation getOrientation() {
        if (!imuExp.isValid()) return imuHub.getOrientation();

        Orientation compositeOrientation = DukConstants.ORIENTATION.NULL_ORIENTATION;
        compositeOrientation.firstAngle = Math.abs(lastOrientation.firstAngle - imuHub.getOrientation().firstAngle)
                < Math.abs(lastOrientation.firstAngle - imuExp.getOrientation().firstAngle)
                ? imuHub.getOrientation().firstAngle
                : imuExp.getOrientation().firstAngle;

        compositeOrientation.secondAngle = Math.abs(lastOrientation.secondAngle - imuHub.getOrientation().secondAngle)
                < Math.abs(lastOrientation.secondAngle - imuExp.getOrientation().secondAngle)
                ? imuHub.getOrientation().secondAngle
                : imuExp.getOrientation().secondAngle;

        compositeOrientation.thirdAngle = Math.abs(lastOrientation.thirdAngle - imuHub.getOrientation().thirdAngle)
                < Math.abs(lastOrientation.thirdAngle - imuExp.getOrientation().thirdAngle)
                ? imuHub.getOrientation().thirdAngle
                : imuExp.getOrientation().thirdAngle;

        lastOrientation = compositeOrientation;
        return compositeOrientation;
    }

    public void setHeading(double heading) {
        imuHub.setHeading(heading);
        imuExp.setHeading(heading);
    }

    @Override
    public void dispatchAllCaches() {
        imuHub.dispatchCache();
        imuExp.dispatchCache();
    }

    @Override
    public void refreshAllCaches() {
        imuHub.refreshCache();
        imuExp.refreshCache();
    }

    @Override
    public void pushTelemetry() {
        loggingBuffer.push("Control Hub Y", imuHub.getOrientation().secondAngle);
        loggingBuffer.push("Control Hub Z", imuHub.getOrientation().thirdAngle);
        loggingBuffer.push("Control Hub X", imuHub.getOrientation().firstAngle);
        if (imuExp.isValid()) {
            loggingBuffer.push("Expansion Hub X", imuExp.getOrientation().firstAngle);
            loggingBuffer.push("Expansion Hub Y", imuExp.getOrientation().secondAngle);
            loggingBuffer.push("Expansion Hub Z", imuExp.getOrientation().thirdAngle);
        }
        Orientation perceivedOrientation = getOrientation();
        loggingBuffer.push("Perceived X", perceivedOrientation.firstAngle);
        loggingBuffer.push("Perceived Y", perceivedOrientation.secondAngle);
        loggingBuffer.push("Perceived Z", perceivedOrientation.thirdAngle);
        loggingBuffer.dispatch();
    }

    @Override
    public void allowDispatch(boolean state) {

    }
}
