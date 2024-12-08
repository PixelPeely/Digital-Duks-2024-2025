package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.assemblies.HardLink;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public class Lift implements CachedSubsystem {
    public PivotDeposit pivotDeposit;
    public HardLink winch;

    public enum STATE {
        DOWN(0, PivotDeposit.STATE.DOWN),
        TRANSFER(0.05, PivotDeposit.STATE.TRANSFER),
        LOW_SPECIMEN(0.2, PivotDeposit.STATE.SPECIMEN_DOWN),
        HIGH_SPECIMEN(0.8, PivotDeposit.STATE.SPECIMEN_DOWN),
        LOW_SAMPLE(0.3, PivotDeposit.STATE.SAMPLE),
        HIGH_SAMPLE(0.9, PivotDeposit.STATE.SAMPLE);

        final double position;
        final PivotDeposit.STATE pivotState;
        STATE(double _position, PivotDeposit.STATE _pivotState) {
            position = DukConstants.HARDWARE.MIN_LIFT_HEIGHT +
                    _position * (DukConstants.HARDWARE.MAX_LIFT_HEIGHT - DukConstants.HARDWARE.MIN_LIFT_HEIGHT);
            pivotState = _pivotState;
        }
    }

    public Lift(HardwareMap hardwareMap) {
        pivotDeposit = new PivotDeposit(hardwareMap);

        winch = new HardLink(
                new HardLink.Link(new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "liftR")), 1),
                new HardLink.Link(new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "liftL")), 1)
        );

        TimeManager.hookTick(t -> {
            winch.setPower(DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PIDF.evaluate(winch.getAveragePower()));
            return false;
        });
    }

    public void setState(STATE state) {
        pivotDeposit.setState(state.pivotState);
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PIDF.target = state.position;
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
        pivotDeposit.pushTelemetry();
    }

    @Override
    public void allowDispatch(boolean state) {
        pivotDeposit.allowDispatch(state);
        winch.allowDispatch(state);
    }
}
