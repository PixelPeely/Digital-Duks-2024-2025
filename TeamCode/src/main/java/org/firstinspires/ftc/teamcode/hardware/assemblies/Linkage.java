package org.firstinspires.ftc.teamcode.hardware.assemblies;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;

public class Linkage implements CachedPeripheral {
    public final HardLink servos;

    private final double retractedAngle;
    private final double extendedAngle;
    private final double positionOffset;
    private double position;

    public Linkage(double _retractedAngle, double _extendedAngle, HardLink _servos) {
        retractedAngle = _retractedAngle;
        extendedAngle = _extendedAngle;
        positionOffset = Math.cos(_retractedAngle);
        servos = _servos;
    }

    public void maxExtension() {
        servos.setPower(1);
    }

    public void setLinearPosition(double _position) {
        double angle = Math.acos((positionOffset + _position) * 0.5);
        double percent = (angle - retractedAngle) / (retractedAngle + extendedAngle);
        servos.setPower(percent);
        position = _position;
    }

    public double getLinearPosition() {
        return positionOffset + position;
    }

    @Override
    public void dispatchCache() {
        servos.dispatchCache();
    }

    @Override
    public void refreshCache() {
        servos.dispatchCache();
    }

    @Override
    public void allowDispatch(boolean state) {
        servos.allowDispatch(state);
    }

    @Override
    public boolean isValid() {
        return servos.isValid();
    }
}
