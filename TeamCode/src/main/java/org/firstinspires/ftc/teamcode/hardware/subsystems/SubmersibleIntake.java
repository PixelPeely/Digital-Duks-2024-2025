package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.assemblies.HardLink;
import org.firstinspires.ftc.teamcode.hardware.assemblies.Linkage;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.InternalTaskInstances;
import org.firstinspires.ftc.teamcode.util.TimeManager;
import org.firstinspires.ftc.teamcode.util.Vector;

public class SubmersibleIntake implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(SubmersibleIntake.class.getSimpleName());

    public final Linkage carriage;
    public final Linkage extendo;
    public final Shuttle shuttle;

    private STATE state = STATE.DROP;
    InternalTaskInstances.SubmersibleIntakeTasks tasks;
    public Lift.STATE desiredLiftState = Lift.STATE.TRANSFER;

    public enum STATE {
        DROP(0, 1, Shuttle.STATE.DROP, true),
        SCOUT(1, 0, Shuttle.STATE.SCOUT, false),
        TRANSFER(0, 0.5, Shuttle.STATE.TRANSFER, true)
        ;

        public final double extendoPosition, carriagePosition;
        public final Shuttle.STATE shuttleState;
        public final boolean closed;
        STATE(double _extendoPosition, double _carriagePosition, Shuttle.STATE _shuttleState, boolean _closed) {
            extendoPosition = _extendoPosition;
            carriagePosition = _carriagePosition;
            shuttleState = _shuttleState;
            closed = _closed;
        }
    }

    public SubmersibleIntake(HardwareMap hardwareMap) {
        C_Servo extendoR = new C_Servo(hardwareMap.tryGet(Servo.class, "extendoR"));
        C_Servo extendoL = new C_Servo(hardwareMap.tryGet(Servo.class, "extendoL"));
        extendoL.setScaleRange(0.05, 0.85);
        extendoR.setScaleRange(0.95, 0.15);
        extendo = new Linkage(
                DukConstants.HARDWARE.EXTENDO_RET_ANGLE,
                DukConstants.HARDWARE.EXTENDO_EXT_ANGLE,
                new HardLink(
                        new HardLink.Link(extendoR, 1),
                        new HardLink.Link(extendoL, 1)
                )
        );
        extendo.servos.setPower(state.extendoPosition);

        C_Servo carriageServo = new C_Servo(hardwareMap.tryGet(Servo.class, "carriage"));
        carriageServo.setScaleRange(0.5, 0.3);
        carriage = new Linkage(
                DukConstants.HARDWARE.CARRIAGE_RET_ANGLE,
                DukConstants.HARDWARE.CARRIAGE_EXT_ANGLE,
                new HardLink(
                        new HardLink.Link(carriageServo, 1)
                )
        );
        carriage.servos.setPower(state.carriagePosition);

        shuttle = new Shuttle(hardwareMap);

        tasks = new InternalTaskInstances.SubmersibleIntakeTasks(this);
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
        if (state == _state) return;
        tasks.cancelAll();
        state = _state;

        shuttle.setState(_state.closed ? Shuttle.STATE.CARRY : Shuttle.STATE.SLIDE);

        if (DukHardwareMap.InternalInteractions.getLiftPosition() < Lift.STATE.EXTENDO_CLEAR.position - DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_ERROR) {
            desiredLiftState = DukHardwareMap.InternalInteractions.getLiftState();
            DukHardwareMap.InternalInteractions.setLiftState(Lift.STATE.EXTENDO_CLEAR);
            TimeManager.hookTick(tasks.liftClearTask);
        } else
            extendoMovement();
    }

    public void extendoMovement() {
        double timeOffset = 0.5;
        TimeManager.hookFuture(timeOffset, tasks.extendoTask);
        timeOffset += 0.5;
        TimeManager.hookFuture(timeOffset, tasks.shuttleTask);
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

        //shuttle.setRoll(rotation);
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
        loggingBuffer.push("Extendo Position", extendo.servos.getAveragePower());
        loggingBuffer.push("Carriage Position", carriage.servos.getAveragePower());
        loggingBuffer.push("State", state.name());
        loggingBuffer.dispatch();
        shuttle.pushTelemetry();
    }

    @Override
    public void allowDispatch(boolean state) {
        carriage.allowDispatch(state);
        extendo.allowDispatch(state);
        shuttle.allowDispatch(state);
    }
}
