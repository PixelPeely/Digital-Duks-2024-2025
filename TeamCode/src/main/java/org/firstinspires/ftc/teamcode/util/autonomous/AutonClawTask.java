package org.firstinspires.ftc.teamcode.util.autonomous;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.TimeManager;

import java.sql.Time;

public class AutonClawTask implements AutonTask{

    private float duration;

    public AutonClawTask(float _duration) {
        duration = _duration;
    }

    @Override
    public void initialize() {
        DukHardwareMap.instance.pixelManagement.setClawIntakePower(-DukConstants.INPUT.CLAW_INTAKE_SPEED);
        duration += TimeManager.getTime(false);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean shouldTerminate() {
        return TimeManager.getTime(false) > duration;
    }

    @Override
    public void onTerminate() {
        DukHardwareMap.instance.pixelManagement.setClawIntakePower(0);
    }

    @Override
    public boolean runAsynchronous() {
        return false;
    }
}
