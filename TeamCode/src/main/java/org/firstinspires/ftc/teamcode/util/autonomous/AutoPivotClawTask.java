package org.firstinspires.ftc.teamcode.util.autonomous;

import static org.firstinspires.ftc.teamcode.util.autonomous.AutoTask.Base._hardwareMap;

public class AutoPivotClawTask implements AutoTask {
    boolean state;

    public AutoPivotClawTask(boolean _state) {
        state = _state;
    }

    @Override
    public void initialize() {
        _hardwareMap.lift.pivotDeposit.claw.setState(state);

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean shouldTerminate() {
        return true;
    }

    @Override
    public void onTerminate() {

    }

    @Override
    public boolean runSynchronous() {
        return false;
    }
}
