package org.firstinspires.ftc.teamcode.hardware.assemblies;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;

public class Linkage implements CachedPeripheral {
    private final C_Servo servo;

    private final double retractedAngle;
    private final double extendedAngle;
    private final double linkageLength;
    private final double positionOffset;
    private double position;

    public Linkage(C_Servo _servo, double _retractedAngle, double _extendedAngle, double _linkageLength) {
        retractedAngle = _retractedAngle;
        extendedAngle = _extendedAngle;
        linkageLength = _linkageLength;
        positionOffset = linkageLength * Math.cos(_retractedAngle);
        servo = _servo;
    }

    public void maxExtension() {
        servo.setPosition(1);
    }

    public void setLinearPosition(double _position) {
        double angle = Math.acos((positionOffset + _position) / (2 * linkageLength));
        double percent = (angle - retractedAngle) / (retractedAngle + extendedAngle);
        servo.setPosition(percent);
        position = _position;
    }

    public double getLinearPosition() {
        return positionOffset + position;
    }

    @Override
    public void dispatchCache() {
        servo.dispatchCache();
    }

    @Override
    public void refreshCache() {
        servo.dispatchCache();
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
