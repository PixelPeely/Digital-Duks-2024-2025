package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.util.autonomous.*;

@Autonomous
public class MySuperCoolAutonomous extends DukAutonomous {
    @Override
    public void buildAutonomous() {
        register(new AutonWaitTask(1));
        register(new AutonPointTask(new Pose(new Vector(0, 10000), 0), 0));
        register(new AutonBranchTask(() -> {
            if (_hardwareMap.driveTrain.poseEstimator.getPose().pos.getY() > 1000)
                return branch(
                        new AutonWaitTask(1),
                        new AutonPointTask(new Pose(new Vector(0, 0), Math.PI / 2), 0)
                );
            return branch();
        }));
    }
}
