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
    public boolean hasElement;

    public enum STATE {
        CARRY(0.5, 0, true),
        DROP(0.5, 0.8, true),
        SLIDE(0.5, 0, false),
        SCOUT(0.5, 0.7, false),
        TRANSFER(0.5, 0, true),
        DEPLOYED(-1, 1, false),
        PICKUP(-1, -1, true),
        PICKUP_CHECK(0.5, 0.7, true)
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
        clawServo.setScaleRange(0.35, 0.65);
        claw = new Claw(clawServo, 0.45, 1);
        claw.setState(state.closed);

        roll = new C_Servo(hardwareMap.tryGet(Servo.class, "shuttleRoll"));
        roll.setScaleRange(0, 1);
        roll.dispatchCache();
        roll.setPosition(state.roll);

        pitch = new C_Servo(hardwareMap.tryGet(Servo.class, "shuttlePitch"));
        pitch.setScaleRange(0.95, 0.2);
        pitch.dispatchCache();
        pitch.setPosition(state.pitch);
    }

    public void setState(STATE _state) {
        if (state == _state) return;
        tasks.cancelAll();
        delayedState = state;
        state = _state;

        if (!state.closed) hasElement = false;

        double offset = 0;
        switch (_state) {
            case DEPLOYED:
                claw.setState(false);
                claw.setPosition(0);
                offset = 0.5;
                TimeManager.hookFuture(offset, tasks.pitchTask);
                break;
            case PICKUP:
                claw.setState(_state.closed);
                hasElement = true;
                offset = 0.5;
                TimeManager.hookFuture(offset, tasks.pickupCheckTask);
                break;
            default:
                //if (_state != STATE.PICKUP_CHECK)
                roll.setPosition(_state.roll);
                pitch.setPosition(_state.pitch);
                claw.setState(_state.closed);
                break;
        }

        System.out.println("Hooked " + _state.name() + " for a time of " + (offset + Math.abs(delayedState.pitch - _state.pitch)));
        TimeManager.hookFuture(offset + Math.abs(delayedState.pitch - _state.pitch) * 1.2, tasks.delayedStateTask);
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
        loggingBuffer.push("Has Element", hasElement);
        loggingBuffer.dispatch();
    }

    @Override
    public void allowDispatch(boolean state) {

    }
}
