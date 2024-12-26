package org.firstinspires.ftc.teamcode.util.autonomous;

import static org.firstinspires.ftc.teamcode.util.autonomous.AutoTask.Base._hardwareMap;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.DukConstants;

public class AutoLiftTask implements AutoTask {
    Lift.STATE state;

    public AutoLiftTask(Lift.STATE _state) {
        state = _state;
    }

    @Override
    public void initialize() {
        _hardwareMap.lift.setState(state);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean shouldTerminate() {
        return Math.abs(_hardwareMap.lift.winch.getAveragePower() - state.position) < DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_ERROR;
    }

    @Override
    public void onTerminate() {

    }

    @Override
    public boolean runSynchronous() {
        return false;
    }
}
