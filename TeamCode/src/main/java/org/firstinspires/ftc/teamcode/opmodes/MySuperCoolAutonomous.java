package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;
import org.firstinspires.ftc.teamcode.util.autonomous.*;

public class MySuperCoolAutonomous extends DukAutonomous{
    @Override
    public void buildAutonomous() {
        register(new AutonBranchTask(() -> {
            if (_hardwareMap.driveTrain.poseEstimator.getPose().x > 10)
                return branch(
                        new AutonWaitTask(1),
                        new AutonPointTask(new Pose(100, 100, (float)Math.PI / 2), 0)
                );
            return branch();
        }));
    }
}
