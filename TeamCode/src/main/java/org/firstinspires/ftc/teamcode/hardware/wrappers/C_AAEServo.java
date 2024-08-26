package org.firstinspires.ftc.teamcode.hardware.wrappers;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;
import org.firstinspires.ftc.teamcode.util.PIDFCalculator;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public class C_AAEServo implements CachedPeripheral {
    public C_CRServo crServo;
    public C_AnalogInput analogInput;
    public PIDFCalculator pidf;

    public C_AAEServo(C_CRServo _crServo, C_AnalogInput _analogInput, PIDFCalculator _pidf) {
        crServo = _crServo;
        analogInput = _analogInput;
        pidf = _pidf;
        TimeManager.hookTick(t -> {
            crServo.setPower(pidf.evaluate(getCurrentPosition()));
            return false;
        });
    }

    public double getCurrentPosition() {
        return analogInput.getScale() * 2 * Math.PI;
    }

    public void setTargetPosition(double target) {
        pidf.target = target;
    }
    
    @Override
    public void dispatchCache() {
        crServo.dispatchCache();
        analogInput.dispatchCache();
    }

    @Override
    public void refreshCache() {
        crServo.refreshCache();
        analogInput.refreshCache();
    }

    @Override
    public void allowDispatch(boolean state) {
        crServo.allowDispatch(state);
        analogInput.allowDispatch(state);
    }

    @Override
    public boolean isValid() {
        return crServo.isValid() && analogInput.isValid();
    }
}
