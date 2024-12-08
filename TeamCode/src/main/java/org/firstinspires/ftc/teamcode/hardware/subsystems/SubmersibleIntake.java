package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.assemblies.HardLink;
import org.firstinspires.ftc.teamcode.hardware.assemblies.Linkage;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.Vector;

public class SubmersibleIntake implements CachedSubsystem {
    public final Linkage carriage;
    public final Linkage extendo;
    public final Shuttle shuttle;

    private STATE state;

    public enum STATE {
        DROP(1, 0, Shuttle.STATE.RETRACTED),
        SCOUT(0, 1, Shuttle.STATE.SCOUT),
        DEPLOYED(0, 1, Shuttle.STATE.DEPLOYED),
        TRANSFER(0.5, 0, Shuttle.STATE.TRANSFER);

        final double carriagePosition, extendoPosition;
        final Shuttle.STATE shuttleState;
        STATE(double _carriagePosition, double _extendoPosition, Shuttle.STATE _shuttleState) {
            carriagePosition = _carriagePosition;
            extendoPosition = _extendoPosition;
            shuttleState = _shuttleState;
        }
    }

    public SubmersibleIntake(HardwareMap hardwareMap) {
        C_Servo extendoR = new C_Servo(hardwareMap.tryGet(Servo.class, "extendoR"));
        C_Servo extendoL = new C_Servo(hardwareMap.tryGet(Servo.class, "extendoL"));
        extendoL.setScaleRange(0.8, 0.05);
        extendoR.setScaleRange(0.2, 0.95);
        extendo = new Linkage(
                DukConstants.HARDWARE.EXTENDO_RET_ANGLE,
                DukConstants.HARDWARE.EXTENDO_EXT_ANGLE,
                new HardLink(
                        new HardLink.Link(extendoR, 1),
                        new HardLink.Link(extendoL, 1)
                )
        );

        C_Servo carriageServo = new C_Servo(hardwareMap.tryGet(Servo.class, "carriage"));
        carriageServo.setScaleRange(0.2, 0.5);
        carriage = new Linkage(
                DukConstants.HARDWARE.CARRIAGE_RET_ANGLE,
                DukConstants.HARDWARE.CARRIAGE_EXT_ANGLE,
                new HardLink(
                        new HardLink.Link(carriageServo, 1)
                )
        );

        shuttle = new Shuttle(hardwareMap);
    }

    public double getLength() {
        return DukConstants.HARDWARE.SUBMERSIBLE_DEAD_LENGTH
                + extendo.getLinearPosition() * DukConstants.HARDWARE.EXTENDO_LINKAGE_LENGTH * 2
                - carriage.getLinearPosition() * DukConstants.HARDWARE.CARRIAGE_LINKAGE_LENGTH * 2;
    }

    public Vector getRelativePosition(double heading) {
        double length = getLength();
        return new Vector(length * Math.sin(heading), length * Math.cos(heading));
    }

    public void setState(STATE _state) {
        state = _state;
        extendo.setLinearPosition(state.extendoPosition);
        carriage.setLinearPosition(state.carriagePosition);
        shuttle.setState(state.shuttleState);
    }

    public STATE getState() {
        return state;
    }

    public void extendToElement(double distance, double rotation) {
        distance -= DukConstants.HARDWARE.SUBMERSIBLE_DEAD_LENGTH;
        double carriagePosition = DukConstants.HARDWARE.CARRIAGE_LINKAGE_LENGTH * 2;
        carriage.setLinearPosition(Math.min(distance / carriagePosition, 1));
        distance -= carriagePosition;
        extendo.setLinearPosition(Math.min(distance / (DukConstants.HARDWARE.EXTENDO_LINKAGE_LENGTH * 2), 1));

        shuttle.setRoll(rotation);
    }

    @Override
    public void dispatchAllCaches() {
        carriage.dispatchCache();
        extendo.dispatchCache();
        shuttle.dispatchAllCaches();
    }

    @Override
    public void refreshAllCaches() {
        carriage.refreshCache();
        extendo.refreshCache();
        shuttle.refreshAllCaches();
    }

    @Override
    public void pushTelemetry() {
        shuttle.pushTelemetry();
    }

    @Override
    public void allowDispatch(boolean state) {
        carriage.allowDispatch(state);
        extendo.allowDispatch(state);
        shuttle.allowDispatch(state);
    }
}
