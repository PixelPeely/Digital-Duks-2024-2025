package org.firstinspires.ftc.teamcode.util.autonomous;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public class AutonWaitTask implements AutonTask {
    private float waitTime;

    public AutonWaitTask(float _waitTime) {
        waitTime = _waitTime;
    }

    @Override
    public void initialize() {
        waitTime += TimeManager.getTime(false);
        DukHardwareMap.instance.driveTrain.applyMagnitude(0);
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
    public boolean runAsynchronous() {
        DashboardInterface.logError("Wait task should not be asynchronous", waitTime);
        return false;
    }
}
