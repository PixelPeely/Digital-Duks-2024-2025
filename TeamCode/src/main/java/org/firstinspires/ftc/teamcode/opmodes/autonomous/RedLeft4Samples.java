package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SubmersibleIntake;
import org.firstinspires.ftc.teamcode.opmodes.DukAutonomous;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.util.autonomous.AutoPickupTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutoPivotClawTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutoLiftTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutoPointTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutoWaitTask;

@Autonomous
public class RedLeft4Samples extends DukAutonomous {
    @Override
    public void buildAutonomous() {
        _hardwareMap.driveTrain.poseEstimator.setPose(new PoseEstimator.Pose(new Vector(29500, -16700), -Math.PI / 2));
        register(new AutoPointTask(new PoseEstimator.Pose(new Vector(22967, -25000), -Math.PI / 4), 500));
        register(new AutoLiftTask(Lift.STATE.HIGH_SAMPLE));
        register(new AutoWaitTask(1));
        register(new AutoPointTask(new PoseEstimator.Pose(new Vector(28000, -29000),-Math.PI / 4), 0));
        register(new AutoPivotClawTask(false));
        register(new AutoWaitTask(1));
        register(new AutoPointTask(new PoseEstimator.Pose(new Vector(22000, -23600), -Math.PI / 2), 0));
        register(new AutoLiftTask(Lift.STATE.TRANSFER_PRIME));
        register(new AutoPickupTask(SubmersibleIntake.STATE.TRANSFER, 0.5));
        register(new AutoWaitTask(2));
        register(new AutoLiftTask(Lift.STATE.HIGH_SAMPLE));
        register(new AutoWaitTask(1));
        register(new AutoPointTask(new PoseEstimator.Pose(new Vector(26700, -29000),-Math.PI / 4), 0));
        register(new AutoPivotClawTask(false));
        register(new AutoWaitTask(1));
        register(new AutoPointTask(new PoseEstimator.Pose(new Vector(22000, -29500), -Math.PI / 2), 0));
        register(new AutoLiftTask(Lift.STATE.TRANSFER_PRIME));
        register(new AutoPickupTask(SubmersibleIntake.STATE.TRANSFER, 0.5));
        register(new AutoWaitTask(2));
        register(new AutoLiftTask(Lift.STATE.HIGH_SAMPLE));
        register(new AutoWaitTask(1));
        register(new AutoPointTask(new PoseEstimator.Pose(new Vector(20841, -28080), -Math.PI / 2), 0));
    }
}
