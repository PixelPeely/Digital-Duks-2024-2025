package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;

public class C_AnalogInput implements CachedPeripheral {
    public final AnalogInput trueAnalogInput;

    private double scale;
    private final double inverseMaxVoltage;

    public C_AnalogInput(AnalogInput analogInput) {
        trueAnalogInput = analogInput;
        inverseMaxVoltage = 1.0 / trueAnalogInput.getMaxVoltage();
    }

    public double getScale() {
        return scale;
    }

    @Override
    public void dispatchCache() {

    }

    @Override
    public void refreshCache() {
        scale = trueAnalogInput.getVoltage() * inverseMaxVoltage;
    }

    @Override
    public void allowDispatch(boolean state) {

    }

    @Override
    public boolean isValid() {
        return trueAnalogInput != null;
    }
}
