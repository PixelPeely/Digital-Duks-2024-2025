package org.firstinspires.ftc.teamcode.util.autonomous;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;

public class AutonWristTask implements AutonTask{
    private boolean deposit;

    public AutonWristTask(boolean _deposit) {
        deposit = _deposit;
    }

    @Override
    public void initialize() {
        DukHardwareMap.instance.pixelManagement.setClawState(deposit);
    }

    @Override
    public void execute() {
        DukHardwareMap.instance.pixelManagement.approachClawState();
    }

    @Override
    public boolean shouldTerminate() {
        return !DukHardwareMap.instance.pixelManagement.wristTraveling &&
                deposit == DukHardwareMap.instance.pixelManagement.wristDepositing;
    }

    @Override
    public void onTerminate() {

    }

    @Override
    public boolean runAsynchronous() {
        return false;
    }
}
