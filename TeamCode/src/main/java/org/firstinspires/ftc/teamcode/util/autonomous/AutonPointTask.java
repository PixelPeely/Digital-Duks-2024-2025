package org.firstinspires.ftc.teamcode.util.autonomous;


import static org.firstinspires.ftc.teamcode.util.autonomous.AutonTask.Base.hMap;

import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.DukUtilities.Vector;

public class AutonPointTask implements AutonTask {
    public final Pose target;
    private final float tolerance;

    public AutonPointTask(Pose _target, float _tolerance) {
        target = _target;
        tolerance = _tolerance + DukConstants.AUTOMATED_CONTROLLER_PARAMS.STANDARD_PURSUIT_RANGE;
    }

    @Override
    public void initialize() {
        hMap.driveTrain.targetPose = target;
        hMap.driveTrain.pursuePosition = hMap.driveTrain.pursueHeading = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean shouldTerminate() {
        return DukUtilities.getDistance(hMap.driveTrain.poseEstimator.getPose().x, hMap.driveTrain.poseEstimator.getPose().y, target.x, target.y)
                < tolerance + DukConstants.AUTOMATED_CONTROLLER_PARAMS.STANDARD_PURSUIT_RANGE &&
                DukUtilities.differenceConstrained(hMap.driveTrain.poseEstimator.getPose().getH(), target.getH())
                < DukConstants.AUTOMATED_CONTROLLER_PARAMS.STANDARD_HEADING_RANGE &&
                hMap.driveTrain.poseEstimator.getPose().s < DukConstants.AUTOMATED_CONTROLLER_PARAMS.MAX_PURSUIT_SPEED &&
                Math.abs(hMap.driveTrain.poseEstimator.getPose().w) < DukConstants.AUTOMATED_CONTROLLER_PARAMS.MAX_PURSUIT_ANGULAR_SPEED;
    }

    @Override
    public void onTerminate() {
        hMap.driveTrain.stopMotors();
        hMap.driveTrain.pursuePosition = hMap.driveTrain.pursueHeading = false;
    }

    @Override
    public boolean runSynchronous() {
        return false;
    }
}
