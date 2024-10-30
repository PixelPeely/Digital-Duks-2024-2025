package org.firstinspires.ftc.teamcode.hardware.assemblies;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;

public class Claw implements CachedPeripheral {
    private final C_Servo servo;
    public boolean closed;

    public Claw(C_Servo _servo) {
        servo = _servo;
    }

    public void setState(boolean _closed) {
        closed = _closed;
        servo.setPosition(_closed ? 1 : 0);
    }

    public void toggle() {
        setState(!closed);
    }

    @Override
    public void dispatchCache() {
        servo.dispatchCache();
    }

    @Override
    public void refreshCache() {
        servo.refreshCache();
    }

    @Override
    public void allowDispatch(boolean state) {
        servo.allowDispatch(state);
    }

    @Override
    public boolean isValid() {
        return servo.isValid();
    }
}
