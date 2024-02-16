package org.firstinspires.ftc.teamcode.util.autonomous;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.TimeManager;

import java.security.Provider;
import java.util.List;
import java.util.function.Supplier;

public class AutonPointTask implements AutonTask{
    private final float x;
    private final float y;
    private final float h;
    private final float pAdd;
    private final float tolerance;

    public AutonPointTask(float _x, float _y, float _h, float _pAdd, float _tolerance) {
        x = _x;
        y = _y;
        h = _h;
        pAdd = _pAdd;
        tolerance = _tolerance + DukConstants.AUTOMATED_CONTROLLER_PARAMS.STANDARD_PURSUIT_RANGE;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        DukHardwareMap.instance.driveTrain.pursueAutonPoint(x, y, pAdd,
                DukHardwareMap.instance.odometerWheels.positionX,
                DukHardwareMap.instance.odometerWheels.positionY,
                DukHardwareMap.instance.odometerWheels.getHeading(true));
        DukHardwareMap.instance.driveTrain.turnTowardsDirectionAbsolute(DukHardwareMap.instance.odometerWheels.getHeading(true), h);
    }

    @Override
    public boolean shouldTerminate() {
        return DukUtilities.getDistance(
                DukHardwareMap.instance.odometerWheels.positionX,
                DukHardwareMap.instance.odometerWheels.positionY,
                x, y) < tolerance + DukConstants.AUTOMATED_CONTROLLER_PARAMS.STANDARD_PURSUIT_RANGE &&
                DukUtilities.differenceConstrained(DukHardwareMap.instance.odometerWheels.getHeading(true), h)
                < DukConstants.AUTOMATED_CONTROLLER_PARAMS.STANDARD_HEADING_RANGE &&
                DukHardwareMap.instance.odometerWheels.totalDeltaET < DukConstants.AUTOMATED_CONTROLLER_PARAMS.MAX_PURSUIT_DELTA_POS_ET &&
                DukHardwareMap.instance.odometerWheels.headingDelta < DukConstants.AUTOMATED_CONTROLLER_PARAMS.MAX_PURSUIT_DELTA_HEADING;
    }

    @Override
    public void onTerminate() {
        DukHardwareMap.instance.driveTrain.applyMagnitude(0);
    }

    @Override
    public boolean runAsynchronous() {
        return false;
    }
}
