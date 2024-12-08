package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;

import java.util.function.Consumer;

public class C_AnalogInput implements CachedPeripheral {
    public final AnalogInput trueAnalogInput;

    private double scale;
    private final double inverseMaxVoltage;

    public Consumer<C_AnalogInput> simRoutine = null;

    public C_AnalogInput(AnalogInput analogInput) {
        trueAnalogInput = analogInput;
        inverseMaxVoltage = analogInput == null ? 0 : 1.0 / trueAnalogInput.getMaxVoltage();
    }

    public double getScale() {
        return scale;
    }

    @Override
    public void dispatchCache() {

    }

    @Override
    public void refreshCache() {
        if (simRoutine == null)
            scale = trueAnalogInput.getVoltage() * inverseMaxVoltage;
        else simRoutine.accept(this);
    }

    @Override
    public void allowDispatch(boolean state) {

    }

    @Override
    public boolean isValid() {
        return trueAnalogInput != null;
    }
}
