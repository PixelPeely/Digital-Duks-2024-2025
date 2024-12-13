package org.firstinspires.ftc.teamcode.util.autonomous;

import static org.firstinspires.ftc.teamcode.util.autonomous.AutoTask.Base._hardwareMap;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shuttle;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SubmersibleIntake;

public class AutoPickupTask implements AutoTask {
    private final SubmersibleIntake.STATE endState;
    private final double roll;
    private boolean hasElement = false;

    public AutoPickupTask(SubmersibleIntake.STATE _endState, double _roll) {
        endState = _endState;
        roll = _roll;
    }

    @Override
    public void initialize() {
        _hardwareMap.submersibleIntake.setState(SubmersibleIntake.STATE.SCOUT);
    }

    @Override
    public void execute() {
        if (_hardwareMap.submersibleIntake.delayedState == SubmersibleIntake.STATE.SCOUT) {
            _hardwareMap.submersibleIntake.shuttle.roll.setPosition(roll);
            _hardwareMap.submersibleIntake.shuttle.setState(Shuttle.STATE.PICKUP);
        }
        if (_hardwareMap.submersibleIntake.shuttle.delayedState == Shuttle.STATE.CARRY) {
            hasElement = true;
            _hardwareMap.submersibleIntake.setState(endState);
        }
    }

    @Override
    public boolean shouldTerminate() {
        return hasElement && _hardwareMap.submersibleIntake.delayedState == endState;
    }

    @Override
    public void onTerminate() {
        DukHardwareMap.InternalInteractions.attemptTransfer();
    }

    @Override
    public boolean runSynchronous() {
        return false;
    }
}
