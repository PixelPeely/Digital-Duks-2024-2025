package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.assemblies.HardLink;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;

public class Clutch implements CachedSubsystem {
    HardLink latch;

    public enum STATE {
        ENGAGED(1),
        FREE(0);

        final double latchPosition;
        STATE(double _latchPosition) {
            latchPosition = _latchPosition;
        }
    }

    public Clutch(HardwareMap hardwareMap) {
        C_Servo clutchL = new C_Servo(hardwareMap.tryGet(Servo.class, "clutchL"));
        C_Servo clutchR = new C_Servo(hardwareMap.tryGet(Servo.class, "clutchR"));

        latch = new HardLink(
                new HardLink.Link(clutchL, 1),
                new HardLink.Link(clutchR, 1)
        );
    }

    public void setState(STATE state) {
        latch.setPower(state.latchPosition);
    }

    @Override
    public void dispatchAllCaches() {
        latch.dispatchCache();
    }

    @Override
    public void refreshAllCaches() {
        latch.refreshCache();
    }

    @Override
    public void pushTelemetry() {

    }

    @Override
    public void allowDispatch(boolean state) {
        latch.allowDispatch(state);
    }
}
