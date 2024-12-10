package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.assemblies.Claw;
import org.firstinspires.ftc.teamcode.hardware.assemblies.HardLink;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_AAEServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_AnalogInput;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_CRServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DukConstants;

public class PivotDeposit implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(PivotDeposit.class.getSimpleName());
    public HardLink pivot;
    public Claw claw;
    public C_Servo yaw;

    private STATE state = STATE.DOWN;
    enum STATE {
        DOWN(0.45, 0.5, true),
        SPECIMEN_DOWN(0, 0.5, true),
        SPECIMEN_RIGHT(0, -0.7, true),
        SPECIMEN_LEFT(0, 0.7, true),
        SAMPLE(1, 0.5, true),
        TRANSFER(0.45, 0.5, false);

        final double pivotPosition, yaw;
        final boolean closed;
        STATE(double _pivotPosition, double _yaw, boolean _closed) {
            pivotPosition = _pivotPosition;
            yaw = _yaw;
            closed = _closed;
        }
    }

    public PivotDeposit(HardwareMap hardwareMap) {
        C_Servo pivotL = new C_Servo(hardwareMap.tryGet(Servo.class, "pivotL"));
        C_Servo pivotR = new C_Servo(hardwareMap.tryGet(Servo.class, "pivotR"));
        pivotL.setScaleRange(0.9, 0.05);
        pivotR.setScaleRange(0.1, 0.95);
        pivot = new HardLink(
                new HardLink.Link(pivotL, 1),
                new HardLink.Link(pivotR, 1)
        );
        pivot.setPower(state.pivotPosition);

        C_Servo clawServo = new C_Servo(hardwareMap.tryGet(Servo.class, "depositClaw"));
        clawServo.setScaleRange(0.2, 0.7);
        claw = new Claw(clawServo);
        claw.setState(state.closed);

        yaw = new C_Servo(hardwareMap.tryGet(Servo.class, "depositYaw"));
        yaw.setScaleRange(0, 1);
        yaw.setPosition(state.yaw);
    }

    public void setState(STATE _state) {
        state = _state;
        pivot.setPower(_state.pivotPosition);
        claw.setState(_state.closed);
        yaw.setPosition(_state.yaw);
    }

    @Override
    public void dispatchAllCaches() {
        pivot.dispatchCache();
        claw.dispatchCache();
        yaw.dispatchCache();
    }

    @Override
    public void refreshAllCaches() {
        pivot.refreshCache();
        claw.refreshCache();
        yaw.refreshCache();
    }

    @Override
    public void pushTelemetry() {
        loggingBuffer.push("Position", pivot.getAveragePower());
        loggingBuffer.push("Claw Closed", claw.closed);
        loggingBuffer.push("Yaw ", yaw.getPosition());
        loggingBuffer.dispatch();
    }

    @Override
    public void allowDispatch(boolean state) {
        pivot.allowDispatch(state);
        claw.allowDispatch(state);
        yaw.allowDispatch(state);
    }
}
