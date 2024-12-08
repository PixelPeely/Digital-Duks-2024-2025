package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;
import org.firstinspires.ftc.teamcode.util.DukConstants;

import java.util.function.Consumer;

public class C_RevDistanceSensor implements CachedPeripheral {
    public final Rev2mDistanceSensor trueSensor;

    private double distance;

    public Consumer<C_RevDistanceSensor> simRoutine = null;

    public C_RevDistanceSensor(Rev2mDistanceSensor sensor) {
        trueSensor = sensor;
    }

    public double getDistance() {
        return distance;
    }

    @Override
    public void dispatchCache() {

    }

    @Override
    public void refreshCache() {
        if (simRoutine == null)
            distance = trueSensor.getDistance(DistanceUnit.MM) * DukConstants.HARDWARE.ET_PER_MM;
        else simRoutine.accept(this);
    }

    @Override
    public void allowDispatch(boolean state) {

    }

    @Override
    public boolean isValid() {
        return trueSensor != null;
    }
}
