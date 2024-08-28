package org.firstinspires.ftc.teamcode.hardware.subsystems;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_AAEServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.TimeManager;
import org.firstinspires.ftc.teamcode.util.Vector;

public class SwerveModule implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer;
    public final C_DcMotor motor;
    public final C_AAEServo servo;
    public final int id;

    private Vector targetState = new Vector();

    public SwerveModule(C_DcMotor _motor, C_AAEServo _servo, int _id) {
        loggingBuffer = new C_TelemetryLoggingBuffer(SwerveModule.class.getSimpleName() + ":" + _id);
        motor = _motor;
        servo = _servo;
        id = _id;
        TimeManager.hookTick(t -> {
            motor.setPower(targetState.getR() * Math.cos(servo.getCurrentPosition() - targetState.getT()));
            return false;
        });
    }

    public Vector getTurningProfile() {
        Vector location = new Vector(DukConstants.HARDWARE.SWERVE_MODULE_POSITIONS[id]);
        location.normalize();
        return new Vector(location.getY(), -location.getX());
    }

    private double getOptimalAzimuth(double pos, double set) {
        double distance = DukUtilities.angleWrap(set - pos);
        double coDistance = DukUtilities.angleWrap(set - pos + Math.PI);
        return Math.abs(distance) < Math.abs(coDistance) ? set : DukUtilities.angleWrap(set + Math.PI);
    }

    public void setTargetState(Vector state) {
        targetState = state;
        servo.setTargetPosition(getOptimalAzimuth(servo.getCurrentPosition(), targetState.getT()));
    }

    @Override
    public void dispatchAllCaches() {
        motor.dispatchCache();
        servo.dispatchCache();
    }

    @Override
    public void refreshAllCaches() {
        motor.refreshCache();
        servo.refreshCache();
    }

    @Override
    public void pushTelemetry() {
        loggingBuffer.push("Motor Power", motor.getPower());
        loggingBuffer.push("Servo Power", servo.getPower());
        loggingBuffer.push("Servo Position", servo.getCurrentPosition());
        loggingBuffer.push("Target Speed", targetState.getR());
        loggingBuffer.push("Target Azimuth", targetState.getT());
    }

    @Override
    public void allowDispatch(boolean state) {

    }
}
