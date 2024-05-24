package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;

public class Hanger implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(Hanger.class.getSimpleName());
    public final C_DcMotor hangerMotorR;
    public final C_DcMotor hangerMotorL;

    public Hanger(HardwareMap hardwareMap) {
        hangerMotorR = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "hangerMotorR"));
        hangerMotorL = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "hangerMotorL"));

        hangerMotorR.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangerMotorL.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangerMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
        hangerMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        hangerMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangerMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangerMotorR.toRefresh[5] = true;
        hangerMotorL.toRefresh[5] = true;

        dispatchAllCaches();
    }

    public void setTarget(float target) {
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_PI.target = (float)DukUtilities.clamp(
                target,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.MAX_HANGER_POSITION,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.MIN_HANGER_POSITION);
    }

    public float getTarget() {
        return DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_PI.target;
    }

    public void approachTarget() {
        float PIDFResult = DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_PI.evaluate(getAveragePosition());

        float max = DukConstants.AUTOMATED_CONTROLLER_PARAMS.MAX_HANGER_POSITION + DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_CUTOFF_TOLERANCE;
        float min = DukConstants.AUTOMATED_CONTROLLER_PARAMS.MIN_HANGER_POSITION + DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_CUTOFF_TOLERANCE;
        if (((hangerMotorR.getCurrentPosition() > max || hangerMotorL.getCurrentPosition() > max) && PIDFResult > 0)
            || ((hangerMotorR.getCurrentPosition() < min || hangerMotorL.getCurrentPosition() < min)) && PIDFResult < 0) {
            setPower(0);
        }
        else {
            setPower(PIDFResult);
        }
    }

    public void setPower(float power) {
        hangerMotorR.setPower(power);
        hangerMotorL.setPower(power);
    }

    public float getAveragePosition() {
        return 0.5f * (hangerMotorR.getCurrentPosition() + hangerMotorL.getCurrentPosition());
    }

    @Override
    public void dispatchAllCaches() {
        hangerMotorR.dispatchCache();
        hangerMotorL.dispatchCache();
    }

    @Override
    public void refreshAllCaches() {
        hangerMotorR.refreshCache();
        hangerMotorL.refreshCache();
    }

    @Override
    public void pushTelemetry() {
        if (!DukConstants.DEBUG.USE_FTC_DASHBOARD) return;

        loggingBuffer.push("Motor Right Power", hangerMotorR.getPower());
        loggingBuffer.push("Motor Left Power", hangerMotorL.getPower());
        loggingBuffer.push("Motor Right Position", hangerMotorR.getCurrentPosition());
        loggingBuffer.push("Motor Left Position", hangerMotorL.getCurrentPosition());
        loggingBuffer.push("Target", DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_PI.target);
        loggingBuffer.dispatch();
    }

    @Override
    public void allowDispatch(boolean state) {
        hangerMotorR.allowDispatch(state);
        hangerMotorL.allowDispatch(state);
    }
}
