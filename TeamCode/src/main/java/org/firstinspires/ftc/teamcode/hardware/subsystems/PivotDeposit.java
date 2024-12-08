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
import org.firstinspires.ftc.teamcode.util.DukConstants;

public class PivotDeposit implements CachedSubsystem {
    public HardLink pivot;
    public Claw claw;
    public C_Servo yaw;

    enum STATE {
        DOWN(0, 0, true),
        SPECIMEN_DOWN(-Math.PI / 2, 0, true),
        SPECIMEN_RIGHT(-Math.PI / 2, -1, true),
        SPECIMEN_LEFT(-Math.PI / 2, 1, true),
        SAMPLE(Math.toRadians(75), 0, true),
        TRANSFER(0, 0, false);

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

        pivotL.setScaleRange(1, 0.2);
        pivotR.setScaleRange(0, 0.8);

        pivot = new HardLink(
                new HardLink.Link(pivotL, 1),
                new HardLink.Link(pivotR, 1)
        );

        C_Servo clawServo = new C_Servo(hardwareMap.tryGet(Servo.class, "depositClaw"));
        clawServo.setScaleRange(0.2, 0.7);
        claw = new Claw(clawServo);

        yaw = new C_Servo(hardwareMap.tryGet(Servo.class, "depositYaw"));
        yaw.setScaleRange(0, 1);
    }

    public void setState(STATE state) {
        pivot.setPower(state.pivotPosition);
        claw.setState(state.closed);
        yaw.setPosition(state.yaw);
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

    }

    @Override
    public void allowDispatch(boolean state) {
        pivot.allowDispatch(state);
        claw.allowDispatch(state);
        yaw.allowDispatch(state);
    }
}
