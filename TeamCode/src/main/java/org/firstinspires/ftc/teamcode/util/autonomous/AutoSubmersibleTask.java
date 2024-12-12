package org.firstinspires.ftc.teamcode.util.autonomous;

import static org.firstinspires.ftc.teamcode.util.autonomous.AutoTask.Base._hardwareMap;

import org.firstinspires.ftc.teamcode.hardware.subsystems.SubmersibleIntake;

public class AutoSubmersibleTask implements AutoTask {
    SubmersibleIntake.STATE state;

    public AutoSubmersibleTask(SubmersibleIntake.STATE _state) {
        state = _state;
    }

    @Override
    public void initialize() {
        _hardwareMap.submersibleIntake.setState(state);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean shouldTerminate() {
        return _hardwareMap.submersibleIntake.delayedState == state;
    }

    @Override
    public void onTerminate() {

    }

    @Override
    public boolean runSynchronous() {
        return false;
    }
}
