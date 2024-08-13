package org.firstinspires.ftc.teamcode.util.autonomous;

import static org.firstinspires.ftc.teamcode.util.autonomous.AutonTask.Base._hardwareMap;

import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public class AutonWaitTask implements AutonTask {
    private double waitTime;

    public AutonWaitTask(double _waitTime) {
        waitTime = _waitTime;
    }

    @Override
    public void initialize() {
        waitTime += TimeManager.getTime(false);
        _hardwareMap.driveTrain.stopMotors();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean shouldTerminate() {
        return TimeManager.getTime(false) > waitTime;
    }

    @Override
    public void onTerminate() {

    }

    @Override
    public boolean runSynchronous() {
        return false;
    }
}
