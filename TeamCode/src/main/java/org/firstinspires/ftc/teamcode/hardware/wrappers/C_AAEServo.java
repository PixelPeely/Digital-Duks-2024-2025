package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;
import org.firstinspires.ftc.teamcode.util.PIDFCalculator;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public class C_AAEServo implements CachedPeripheral {
    public C_CRServo crServo;
    public C_AnalogInput analogInput;
    public PIDFCalculator pidf;

    private boolean invertRefresh;

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
        return (analogInput.getScale() - 0.5) * 2 * Math.PI * (invertRefresh ? -1 : 1);
    }

    public void setTargetPosition(double target) {
        pidf.target = target;
    }

    //region True
    public double getPower() {
        return crServo.getPower();
    }

    public void setDirection(CRServo.Direction direction) {
        crServo.setDirection(direction);
        invertRefresh = direction == DcMotorSimple.Direction.REVERSE;
    }

    public CRServo.Direction getDirection() {
        return crServo.getDirection();
    }
    //endregion

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
