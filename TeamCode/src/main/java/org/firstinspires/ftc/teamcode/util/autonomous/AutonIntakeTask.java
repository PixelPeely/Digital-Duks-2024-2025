package org.firstinspires.ftc.teamcode.util.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.TimeManager;

import java.sql.Time;

public class AutonIntakeTask implements AutonTask {
    private final boolean reversed;
    private double duration;

    public AutonIntakeTask(boolean _reversed, float _duration) {
        reversed = _reversed;
        duration = _duration;
    }

    @Override
    public void initialize() {
        DukHardwareMap.instance.pixelManagement.toggleIntake(reversed);
        DukHardwareMap.instance.driveTrain.moveDirectionRelative(Math.PI);
        DukHardwareMap.instance.driveTrain.applyMagnitude(0.2f);
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
        DukHardwareMap.instance.pixelManagement.toggleIntake(false);
        DukHardwareMap.instance.driveTrain.applyMagnitude(0);
    }

    @Override
    public boolean runAsynchronous() {
        return false;
    }
}
