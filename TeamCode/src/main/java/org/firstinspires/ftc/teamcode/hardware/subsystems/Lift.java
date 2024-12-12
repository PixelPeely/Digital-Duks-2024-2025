package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.assemblies.HardLink;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.InternalTaskInstances;
import org.firstinspires.ftc.teamcode.util.TimeManager;

import java.util.function.Predicate;

public class Lift implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(Lift.class.getSimpleName());
    private final InternalTaskInstances.LiftTasks tasks;

    public PivotDeposit pivotDeposit;
    public HardLink winch;
    private final C_DcMotor liftR, liftL;

    private STATE state = STATE.DOWN;
    private STATE desiredState;
    private boolean extendoRetracted = true;

    public enum STATE {
        DOWN(0, PivotDeposit.STATE.DOWN),
        TRANSFER(0, PivotDeposit.STATE.TRANSFER),
        LOW_SPECIMEN(0.4, PivotDeposit.STATE.SPECIMEN_DOWN),
        HIGH_SPECIMEN(0.5, PivotDeposit.STATE.SPECIMEN_DOWN),
        LOW_SAMPLE(0.4, PivotDeposit.STATE.SAMPLE),
        HIGH_SAMPLE(0.7, PivotDeposit.STATE.SAMPLE),
        EXTENDO_CLEAR(0.4, PivotDeposit.STATE.DOWN),
        ;

        public final double position;
        public final PivotDeposit.STATE pivotState;
        STATE(double _position, PivotDeposit.STATE _pivotState) {
            position = DukConstants.HARDWARE.MIN_LIFT_HEIGHT +
                    _position * (DukConstants.HARDWARE.MAX_LIFT_HEIGHT - DukConstants.HARDWARE.MIN_LIFT_HEIGHT);
            pivotState = _pivotState;
        }
    }

    public Lift(HardwareMap hardwareMap) {
        pivotDeposit = new PivotDeposit(hardwareMap);

        liftR = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "liftR"));
        liftL = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "liftL"));

        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.dispatchCache();
        liftL.dispatchCache();

        liftL.toRefresh[5] = true;
        liftR.toRefresh[5] = true;

        winch = new HardLink(
                new HardLink.Link(liftR, 1),
                new HardLink.Link(liftL, 1)
        );

        TimeManager.hookTick(t -> {
            winch.setPower(DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PIDF.evaluate(winch.getAveragePower()));
            return false;
        });

        tasks = new InternalTaskInstances.LiftTasks(this);
    }

    public void manualAdjustment(double magnitude) {
        if (magnitude == 0) return;
        setLiftPosition(winch.getAveragePower() + magnitude * DukConstants.INPUT.MANUAL_LIFT_SPEED);
    }

    public void setLiftPosition(double position) {
        if (!extendoRetracted && position < STATE.EXTENDO_CLEAR.position) return;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PIDF.target = DukUtilities.clamp(position,
                DukConstants.HARDWARE.MAX_LIFT_HEIGHT, DukConstants.HARDWARE.MIN_LIFT_HEIGHT);
    }

    public void setState(STATE _state) {
        desiredState = _state;
        if (desiredState.position < STATE.EXTENDO_CLEAR.position && !extendoRetracted) return;

        if (state == _state) return;
        state = _state;
        tasks.cancelAll();
        setLiftPosition(_state.position);

        pivotDeposit.claw.setState(state.pivotState.closed);
        pivotDeposit.yaw.setPosition(state.pivotState.yaw);
        TimeManager.hookTick(tasks.setPivotState);

        if (state == STATE.TRANSFER) TimeManager.hookTick(tasks.requestTransfer);
    }

    public STATE getState() {
        return state;
    }

    public boolean canTransfer() {
        return getState() == STATE.TRANSFER
                && Math.abs(winch.getAveragePower() - STATE.TRANSFER.position) < DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_ERROR;
    }

    public void extendoClearanceUpdate(boolean retracted) {
        if (retracted == extendoRetracted) return;
        extendoRetracted = retracted;

        if (retracted)
            setState(desiredState);
        else if (getState().position < STATE.EXTENDO_CLEAR.position) {
            STATE currentState = getState();
            setState(STATE.EXTENDO_CLEAR);
            desiredState = currentState;
        }
    }

    public void resetEncoders() {
        liftL.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void dispatchAllCaches() {
        pivotDeposit.dispatchAllCaches();
        winch.dispatchCache();
    }

    @Override
    public void refreshAllCaches() {
        pivotDeposit.refreshAllCaches();
        winch.refreshCache();
    }

    @Override
    public void pushTelemetry() {
        loggingBuffer.push("Target ", DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PIDF.target);
        loggingBuffer.push("Position ", winch.getAveragePower());
        loggingBuffer.push("State ", state.name());
        loggingBuffer.dispatch();
        pivotDeposit.pushTelemetry();
    }

    @Override
    public void allowDispatch(boolean state) {
        pivotDeposit.allowDispatch(state);
        winch.allowDispatch(state);
    }
}
