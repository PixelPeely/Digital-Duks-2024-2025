package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.assemblies.Claw;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.InternalTaskInstances;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public class Shuttle implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(Shuttle.class.getSimpleName());
    private final InternalTaskInstances.ShuttleTasks tasks;

    public final Claw claw;
    public final C_Servo roll;
    public final C_Servo pitch;

    private STATE state = STATE.DROP;
    public STATE delayedState = state;
    private double rollAngle;

    public enum STATE {
        CARRY(0.5, 0, true),
        DROP(0.5, 0.8, true),
        SLIDE(0.5, 0, false),
        SCOUT(0.5, 0.6, false),
        TRANSFER(0.5, 0, true),
        PICKUP(-1, 0.8, true),
        PICKUP_CHECK(-1, 0.6, true),
        ;

        public final double roll, pitch;
        public final boolean closed;
        STATE(double _roll, double _pitch, boolean _closed) {
            roll = _roll;
            pitch = _pitch;
            closed = _closed;
        }
    }

    public Shuttle(HardwareMap hardwareMap) {
        tasks = new InternalTaskInstances.ShuttleTasks(this);

        C_Servo clawServo = new C_Servo(hardwareMap.tryGet(Servo.class, "shuttleClaw"));
        clawServo.setScaleRange(0.45, 0.65);
        claw = new Claw(clawServo);
        claw.setState(state.closed);

        roll = new C_Servo(hardwareMap.tryGet(Servo.class, "shuttleRoll"));
        roll.setScaleRange(0, 1);
        roll.setPosition(state.roll);

        pitch = new C_Servo(hardwareMap.tryGet(Servo.class, "shuttlePitch"));
        pitch.setScaleRange(0.95, 0);
        pitch.setPosition(state.pitch);
    }

    public void setState(STATE _state) {
        if (state == _state) return;
        tasks.cancelAll();
        delayedState = state;
        TimeManager.hookFuture(Math.abs(state.pitch - _state.pitch) * 0.5, tasks.delayedStateTask);
        state = _state;

        if (_state == STATE.PICKUP) {
            claw.setState(false);
            double timeOffset = 0.5;
            TimeManager.hookFuture(timeOffset, tasks.pitchTask);
            timeOffset += 0.75;
            TimeManager.hookFuture(timeOffset, tasks.pickupTask);
            timeOffset += 0.25;
            TimeManager.hookFuture(timeOffset, tasks.pickupCheckTask);
        } else {
            if (_state != STATE.PICKUP_CHECK)
                roll.setPosition(_state.roll);
            pitch.setPosition(_state.pitch);
            claw.setState(_state.closed);
        }
    }

    public STATE getState() {
        return state;
    }
    //
//    public void setRoll(double angle) {
//        roll.setPosition(angle * (1 / (Math.PI * 2)));
//        rollAngle = angle;
//    }

//    public double getRoll() {
//        return rollAngle;
//    }

    @Override
    public void dispatchAllCaches() {
        claw.dispatchCache();
        roll.dispatchCache();
        pitch.dispatchCache();
    }

    @Override
    public void refreshAllCaches() {
        claw.refreshCache();
        roll.refreshCache();
        pitch.refreshCache();
    }

    @Override
    public void pushTelemetry() {
        loggingBuffer.push("Claw Closed", claw.closed);
        loggingBuffer.push("Roll", roll.getPosition());
        loggingBuffer.push("Pitch", pitch.getPosition());
        loggingBuffer.push("State", state);
        loggingBuffer.push("Delayed State", delayedState);
        loggingBuffer.dispatch();
    }

    @Override
    public void allowDispatch(boolean state) {

    }
}
