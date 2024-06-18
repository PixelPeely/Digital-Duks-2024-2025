package org.firstinspires.ftc.teamcode.util.autonomous;


import static org.firstinspires.ftc.teamcode.util.autonomous.AutonTask.Base._hardwareMap;

import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;

public class AutonPointTask implements AutonTask {
    public final Pose target;
    private final float tolerance;

    public AutonPointTask(Pose _target, float _tolerance) {
        target = _target;
        tolerance = _tolerance + DukConstants.AUTOMATED_CONTROLLER_PARAMS.STANDARD_PURSUIT_RANGE;
    }

    @Override
    public void initialize() {
        _hardwareMap.driveTrain.targetPose = target;
        _hardwareMap.driveTrain.pursuePosition = _hardwareMap.driveTrain.pursueHeading = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean shouldTerminate() {
        return DukUtilities.getDistance(_hardwareMap.driveTrain.poseEstimator.getPose().x, _hardwareMap.driveTrain.poseEstimator.getPose().y, target.x, target.y)
                < tolerance + DukConstants.AUTOMATED_CONTROLLER_PARAMS.STANDARD_PURSUIT_RANGE &&
                DukUtilities.differenceConstrained(_hardwareMap.driveTrain.poseEstimator.getPose().getH(), target.getH())
                < DukConstants.AUTOMATED_CONTROLLER_PARAMS.STANDARD_HEADING_RANGE &&
                _hardwareMap.driveTrain.poseEstimator.getPose().s < DukConstants.AUTOMATED_CONTROLLER_PARAMS.MAX_PURSUIT_SPEED &&
                Math.abs(_hardwareMap.driveTrain.poseEstimator.getPose().w) < DukConstants.AUTOMATED_CONTROLLER_PARAMS.MAX_PURSUIT_ANGULAR_SPEED;
    }

    @Override
    public void onTerminate() {
        _hardwareMap.driveTrain.stopMotors();
        _hardwareMap.driveTrain.pursuePosition = _hardwareMap.driveTrain.pursueHeading = false;
    }

    @Override
    public boolean runSynchronous() {
        return false;
    }
}
