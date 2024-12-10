package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;

import java.util.Arrays;

public class C_Servo implements CachedPeripheral {
    public final Servo trueServo;

    private Servo.Direction direction;
    private double position = Double.NaN;
    private double[] scaleRange = new double[2];

    private double targetPosition;
    private byte invertScaleRange;

    public boolean[] toDispatch = new boolean[3];
    public boolean[] toRefresh = new boolean[2];
    private boolean allowDispatch = true;

    public C_Servo (Servo servo) {
        if (servo == null) System.out.println("C_Servo is null, running as dummy!");
        trueServo = servo;
    }

    public Servo.Direction getDirection() {return direction;}
    public double getPosition() {return position;}

    public double getTargetPosition() {return targetPosition;}
    public double[] getScaleRange() {return scaleRange;}

    public void setDirection(Servo.Direction _direction) {
        if (_direction == direction) return;
        toDispatch[0] = true;
        direction = _direction;
    }
    public void setPosition(double _position) {
        if (_position == position) return;
        toDispatch[1] = true;
        position = _position;
    }

    public void setTargetPosition(double _position) {
        targetPosition = _position;
    }
    public void setScaleRange(double min, double max) {
        if (scaleRange[0] == min && scaleRange[0] == max) return;
        toDispatch[2] = true;
        if (min <= max) {
            scaleRange[0] = min;
            scaleRange[1] = max;
            invertScaleRange = 0;
        } else {
            scaleRange[0] = max;
            scaleRange[1] = min;
            invertScaleRange = 1;
        }
    }

    @Override
    public void dispatchCache() {
        if (trueServo == null || !allowDispatch) return;
        if (toDispatch[0]) trueServo.setDirection(direction);
        if (toDispatch[2]) trueServo.scaleRange(scaleRange[0], scaleRange[1]);
        if (toDispatch[1]) trueServo.setPosition(Math.abs(invertScaleRange - position));
        Arrays.fill(toDispatch, false);
    }

    @Override
    public void refreshCache() {
        if (trueServo == null) return;
        if (toRefresh[0]) direction = trueServo.getDirection();
        if (toRefresh[1]) position = trueServo.getPosition();
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

    public void approachTargetPosition(double speed) {
        setPosition(position + (targetPosition - position) * speed);
        //setPosition(Math.abs(position - targetPosition) > Math.abs(speed) ? position + targetPosition * speed : targetPosition);
    }
}
