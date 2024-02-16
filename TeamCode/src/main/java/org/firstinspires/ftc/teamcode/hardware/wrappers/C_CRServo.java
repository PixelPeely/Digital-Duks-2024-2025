package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;

import java.util.Arrays;

public class C_CRServo implements CachedPeripheral {
    public final CRServo trueServo;

    private CRServo.Direction direction;
    private double power;

    public boolean[] toDispatch = new boolean[3];
    public boolean[] toRefresh = new boolean[2];
    private boolean allowDispatch = true;

    public C_CRServo(CRServo servo) {
        if (servo == null) System.out.println("C_CRServo is null, running as dummy!");
        trueServo = servo;
    }

    public CRServo.Direction getDirection() {return direction;}
    public double getPower() {return power;}

    public void setDirection(CRServo.Direction _direction) {
        if (_direction == direction) return;
        toDispatch[0] = true;
        direction = _direction;
    }
    public void setPower(double _power) {
        if (_power == power) return;
        toDispatch[1] = true;
        power = _power;
    }

    @Override
    public void dispatchCache() {
        if (trueServo == null || !allowDispatch) return;
        if (toDispatch[0]) trueServo.setDirection(direction);
        if (toDispatch[1]) trueServo.setPower(power);
        Arrays.fill(toDispatch, false);
    }

    @Override
    public void refreshCache() {
        if (trueServo == null) return;
        if (toRefresh[0]) direction = trueServo.getDirection();
        if (toRefresh[1]) power = trueServo.getPower();
        Arrays.fill(toDispatch, false);                      
    }

    @Override
    public void allowDispatch(boolean state) {
        allowDispatch = state;
    }

    @Override
    public boolean isValid() {
        return trueServo != null;
    }
}
