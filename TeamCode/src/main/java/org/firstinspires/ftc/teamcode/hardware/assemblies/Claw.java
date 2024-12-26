package org.firstinspires.ftc.teamcode.hardware.assemblies;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;

public class Claw implements CachedPeripheral {
    private final C_Servo servo;
    public boolean closed;

    private final double openPos;
    private final double closedPos;

    public Claw(C_Servo _servo) {
        openPos = 0;
        closedPos = 1;
        servo = _servo;
    }

    public Claw(C_Servo _servo, double _openPos, double _closedPos) {
        openPos = _openPos;
        closedPos = _closedPos;
        servo = _servo;
    }

    public void setState(boolean _closed) {
        closed = _closed;
        servo.setPosition(_closed ? closedPos : openPos);
        dispatchCache();
    }

    public void setPosition(double position) {
        servo.setPosition(position);
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
