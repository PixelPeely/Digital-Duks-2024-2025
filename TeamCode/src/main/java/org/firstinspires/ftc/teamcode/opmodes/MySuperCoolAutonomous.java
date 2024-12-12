package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.util.autonomous.*;

@Autonomous
public class MySuperCoolAutonomous extends DukAutonomous {
    @Override
    public void buildAutonomous() {
        register(new AutoWaitTask(1));
        register(new AutoPointTask(new Pose(new Vector(0, 10000), 0), 0));
        register(new AutoBranchTask(() -> {
            if (_hardwareMap.driveTrain.poseEstimator.getPose().pos.getY() > 1000)
                return branch(
                        new AutoWaitTask(1),
                        new AutoPointTask(new Pose(new Vector(), Math.PI / 2), 0)
                );
            return branch();
        }));
    }
}
