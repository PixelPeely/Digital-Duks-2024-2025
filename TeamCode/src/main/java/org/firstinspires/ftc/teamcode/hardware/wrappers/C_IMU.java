package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;
import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;

public class C_IMU implements CachedPeripheral {
    private final IMU trueImu;

    private Orientation orientation = DukConstants.ORIENTATION.NULL_ORIENTATION;
    private float headingOffset = 0;
    private AngularVelocity angularVelocity;
    private YawPitchRollAngles yawPitchRollAngles;

    public boolean[] toRefresh = new boolean[3];
    public boolean invertRefresh = false;

    public AxesReference axesReference;
    public AxesOrder axesOrder;
    public AngleUnit angleUnit;

    public C_IMU(IMU imu) {
        if (imu == null) DashboardInterface.logError("C_IMU is null, running as dummy!", null);
        trueImu = imu;
    }

    public void initialize(IMU.Parameters _parameters) {
        if (trueImu == null) return;
        trueImu.initialize(_parameters);
    }

    public Orientation getOrientation() {
        Orientation finalOrientation = DukConstants.ORIENTATION.NULL_ORIENTATION;
        finalOrientation.firstAngle = orientation.firstAngle;
        finalOrientation.secondAngle = orientation.secondAngle;
        finalOrientation.thirdAngle = orientation.thirdAngle - headingOffset;
        return finalOrientation;
    }
    public AngularVelocity getAngularVelocity() {return angularVelocity;}
    public YawPitchRollAngles getYawPitchRollAngles() {return yawPitchRollAngles;}

    public void setHeading(float heading) {
        refreshCache();
        headingOffset = getOrientation().thirdAngle - heading;
        DashboardInterface.logError("Zeroed at ", headingOffset);
    }

    @Override
    public void dispatchCache() {

    }

    @Override
    public void refreshCache() {
        if (trueImu == null) return;
        if (toRefresh[0]) orientation = trueImu.getRobotOrientation(axesReference, axesOrder, angleUnit);
        if (toRefresh[1]) angularVelocity = trueImu.getRobotAngularVelocity(angleUnit);
        if (toRefresh[2]) yawPitchRollAngles = trueImu.getRobotYawPitchRollAngles();
        if (invertRefresh) {
            if (toRefresh[0]) {
                orientation.firstAngle *= -1;
                orientation.secondAngle *= -1;
                orientation.thirdAngle *= -1;
            }
            if (toRefresh[1]) {
                angularVelocity.xRotationRate *= -1;
                angularVelocity.yRotationRate *= -1;
                angularVelocity.zRotationRate *= -1;
            }
        }
    }

    @Override
    public void allowDispatch(boolean state) {

    }

    @Override
    public boolean isValid() {
        return trueImu != null;
    }
}
