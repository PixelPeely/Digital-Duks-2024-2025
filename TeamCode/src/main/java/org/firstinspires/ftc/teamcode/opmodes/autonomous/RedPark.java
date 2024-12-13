package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator;
import org.firstinspires.ftc.teamcode.opmodes.DukAutonomous;
import org.firstinspires.ftc.teamcode.opmodes.DukOpMode;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.util.autonomous.AutoPointTask;

@Autonomous
public class RedPark extends DukAutonomous {
    @Override
    public void buildAutonomous() {
        _hardwareMap.driveTrain.poseEstimator.setPose(new PoseEstimator.Pose(new Vector(61000, -35000), -Math.PI / 2));
        register(new AutoPointTask(new PoseEstimator.Pose(new Vector(61000, -60000), -Math.PI / 2), 0));
    }
}
