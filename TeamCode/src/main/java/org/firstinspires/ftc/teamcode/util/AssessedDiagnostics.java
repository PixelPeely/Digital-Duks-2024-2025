package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;

@Config
//All time units are in seconds
public class AssessedDiagnostics {
    private final DukHardwareMap hMap;

    public static boolean assessOdometers = true;

    //FT = Failure Timer
    //-1 = No failures detected
    //-2 = Failure detected, system shut down
    private double odometerDirectionFT = -1;
    private double odometerHeadingFT = -1;

    private float lastHeadingDifference;
    private double lastIMUCheck;

    public boolean odometersFailed;

    public AssessedDiagnostics(DukHardwareMap _hMap) {
        hMap = _hMap;
    }

    public void assessPassive() {
        if (odometerDirectionFT != -2 && odometerHeadingFT != -2 && assessOdometers) assessOdometersPassive();

        checkTimers();
    }

    private void checkTimers() {
        if ((odometerDirectionFT > -1 || odometerHeadingFT > -1)
                && TimeManager.getTime(false) - Math.min(odometerDirectionFT, odometerHeadingFT)
                    > DukConstants.ASSESSED_DIAGNOSTICS.MAX_ODOMETER_FAILURE_TIME)
            onOdometersFail();
    }

    private void onOdometersFail() {
//        odometersFailed = true;
//        DashboardInterface.logError(AssessedDiagnostics.class.getSimpleName() + ": Odometers failed",
//                odometerDirectionFT == -2 ? "Direction" : odometerHeadingFT == -2 ? "Heading" : "IMU Deviation");
//        odometerDirectionFT = -2;
//        odometerHeadingFT = -2;
    }

    private void assessOdometersPassive() {
        if (hMap.driveTrain.getMotorPowerSum() > DukConstants.ASSESSED_DIAGNOSTICS.MIN_DRIVE_POWER) {
            //Are we moving in the requested direction?
            float perceivedMovementDirection = (float) Math.atan2(hMap.odometerWheels.positionDeltaX, hMap.odometerWheels.positionDeltaY);
            if (Math.abs(perceivedMovementDirection - hMap.driveTrain.lastRequestedMovementDirection)
                    > DukConstants.ASSESSED_DIAGNOSTICS.ODOMETER_MAX_DIRECTION_DEVIATION)
                odometerDirectionFT = TimeManager.getTime(false);
            else
                odometerDirectionFT = -1;

            //Is the heading changing towards the target orientation?
            float thisHeadingDifference = Math.abs(DukUtilities.differenceConstrained(hMap.odometerWheels.getHeading(false), hMap.driveTrain.lastRequestedTurnDirection));
            if (thisHeadingDifference > lastHeadingDifference)
                odometerHeadingFT = TimeManager.getTime(false);
            else
                odometerHeadingFT = -1;
            lastHeadingDifference = thisHeadingDifference;
        }
        //Does the heading match that of the IMU? If the difference is large enough, update the odometer heading,
        //otherwise, increment the failure counter
        if (TimeManager.getTime(false) - lastIMUCheck > DukConstants.ASSESSED_DIAGNOSTICS.IMU_CHECK_INTERVAL) {
            lastIMUCheck = TimeManager.getTime(false);
            hMap.duplexIMU.refreshAllCaches();
            if (Math.abs(DukUtilities.differenceConstrained(hMap.odometerWheels.getHeading(false), hMap.duplexIMU.getOrientation().thirdAngle))
                    > DukConstants.ASSESSED_DIAGNOSTICS.MAX_IMU_ODOMETER_DIFFERENCE)
                onOdometersFail();
            else
                hMap.odometerWheels.setRelativeHeading(hMap.duplexIMU.getOrientation().thirdAngle);
        }
    }
}
