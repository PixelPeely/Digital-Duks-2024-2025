package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;

import java.util.Arrays;

public class C_DcMotor implements CachedPeripheral {
    public final DcMotorEx trueDcMotor;

    private double power;
    private DcMotorSimple.Direction direction;
    private DcMotorEx.RunMode runMode;
    private int targetPosition;
    private DcMotorEx.ZeroPowerBehavior zeroPowerBehavior;
    private int currentPosition;
    private double current;

    private boolean[] toDispatch = new boolean[5];
    public boolean[] toRefresh = new boolean[7];
    public boolean invertRefresh = false;
    private boolean allowDispatch = true;

    public C_DcMotor(DcMotorEx dcMotor) {
        if (dcMotor == null) System.out.println("C_DcMotor is null, running as dummy!");
        trueDcMotor = dcMotor;
    }

    public void setPower(double _power) {
        if (_power == power) return;
        toDispatch[0] = true;
        power = _power;
    }
    public void setDirection(DcMotorSimple.Direction _direction) {
        if (_direction == direction) return;
        toDispatch[1] = true;
        direction = _direction;
    }
    public void setRunMode(DcMotorEx.RunMode _runMode) {
        if (_runMode == runMode) return;
        toDispatch[2] = true;
        runMode = _runMode;
    }
    public void setTargetPosition(int _targetPosition) {
        if (_targetPosition == targetPosition) return;
        toDispatch[3] = true;
        targetPosition = _targetPosition;
    }
    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior _zeroPowerBehavior) {
        if (_zeroPowerBehavior == zeroPowerBehavior) return;
        toDispatch[4] = true;
        zeroPowerBehavior = _zeroPowerBehavior;
    }

    public double getPower() {return power;}
    public DcMotorSimple.Direction getDirection() {return direction;}
    public DcMotorEx.RunMode getRunMode() {return runMode;}
    public int getTargetPosition() {return targetPosition;}
    public DcMotorEx.ZeroPowerBehavior getZeroPowerBehavior() {return zeroPowerBehavior;}
    public int getCurrentPosition() {return currentPosition * (invertRefresh ? -1:1);}
    public double getCurrent() {return current;}

    @Override
    public void dispatchCache() {
        if (trueDcMotor == null || !allowDispatch) return;
        if (toDispatch[0]) trueDcMotor.setPower(power);
        if (toDispatch[1]) trueDcMotor.setDirection(direction);
        if (toDispatch[3]) trueDcMotor.setTargetPosition(targetPosition);
        if (toDispatch[2]) trueDcMotor.setMode(runMode);
        if (toDispatch[4]) trueDcMotor.setZeroPowerBehavior(zeroPowerBehavior);
        Arrays.fill(toDispatch, false);
    }

    @Override
    public void refreshCache() {
        if (trueDcMotor == null) return;
        if (toRefresh[0]) power = trueDcMotor.getPower();
        if (toRefresh[1]) direction = trueDcMotor.getDirection();
        if (toRefresh[2]) runMode = trueDcMotor.getMode();
        if (toRefresh[3]) targetPosition = trueDcMotor.getTargetPosition();
        if (toRefresh[4]) zeroPowerBehavior = trueDcMotor.getZeroPowerBehavior();
        if (toRefresh[5]) currentPosition = trueDcMotor.getCurrentPosition();
        if (toRefresh[6]) current = trueDcMotor.getCurrent(CurrentUnit.AMPS);
        Arrays.fill(toDispatch, false);
    }

    @Override
    public void allowDispatch(boolean state) {
        allowDispatch = state;
    }

    @Override
    public boolean isValid() {
        return trueDcMotor != null;
    }
}