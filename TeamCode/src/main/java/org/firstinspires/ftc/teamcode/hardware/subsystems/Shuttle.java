package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.assemblies.Claw;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;

public class Shuttle implements CachedSubsystem {
    public final Claw claw;
    private final C_Servo roll;
    private final C_Servo pitch;

    private double rollAngle;

    enum STATE {
        DEPLOYED(0, 1, false),
        SCOUT(0, 0, false),
        TRANSFER(0, 0, true),
        RETRACTED(0, 0, true);

        final double roll, pitch;
        final boolean closed;
        STATE(double _roll, double _pitch, boolean _closed) {
            roll = _roll;
            pitch = _pitch;
            closed = _closed;
        }
    }

    public Shuttle(HardwareMap hardwareMap) {
        C_Servo clawServo = new C_Servo(hardwareMap.tryGet(Servo.class, "shuttleClaw"));
        clawServo.setScaleRange(0.2, 0.5);
        claw = new Claw(clawServo);

        roll = new C_Servo(hardwareMap.tryGet(Servo.class, "shuttleRoll"));
        roll.setScaleRange(0, 1);

        pitch = new C_Servo(hardwareMap.tryGet(Servo.class, "shuttlePitch"));
        pitch.setScaleRange(0, 1);
    }

    public void setState(STATE state) {
        setRoll(state.roll);
        pitch.setPosition(state.pitch);
        claw.setState(state.closed);
    }

    public void setRoll(double angle) {
        roll.setPosition(angle * (1 / (Math.PI * 2)));
    }

    public double getRoll() {
        return rollAngle;
    }

    @Override
    public void dispatchAllCaches() {

    }

    @Override
    public void refreshAllCaches() {

    }

    @Override
    public void pushTelemetry() {

    }

    @Override
    public void allowDispatch(boolean state) {

    }
}
