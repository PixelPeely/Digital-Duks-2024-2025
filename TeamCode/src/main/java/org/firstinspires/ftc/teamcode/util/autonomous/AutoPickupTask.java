package org.firstinspires.ftc.teamcode.util.autonomous;

import static org.firstinspires.ftc.teamcode.util.autonomous.AutoTask.Base._hardwareMap;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shuttle;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SubmersibleIntake;

public class AutoPickupTask implements AutoTask {
    private final SubmersibleIntake.STATE endState;
    private final double roll;

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
        if (_hardwareMap.submersibleIntake.delayedState != SubmersibleIntake.STATE.SCOUT) return;
        switch (_hardwareMap.submersibleIntake.shuttle.delayedState) {
            case SCOUT:
                _hardwareMap.submersibleIntake.shuttle.roll.setPosition(roll);
                _hardwareMap.submersibleIntake.shuttle.setState(Shuttle.STATE.DEPLOYED);
                break;
            case DEPLOYED:
                _hardwareMap.submersibleIntake.shuttle.setState(Shuttle.STATE.PICKUP);
                break;
            case PICKUP_CHECK:
                System.out.println("Pickup check complete");
                _hardwareMap.submersibleIntake.setState(endState);
                break;
        }
    }

    @Override
    public boolean shouldTerminate() {
        return _hardwareMap.submersibleIntake.shuttle.hasElement && _hardwareMap.submersibleIntake.delayedState == endState;
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
