package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_CRServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public class PixelManagement implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(PixelManagement.class.getSimpleName());
    public final C_DcMotor intakeRoller;
    public final C_DcMotor lift;
    public final C_Servo wristPivot;
    public final C_Servo clawPivot;
    public final C_CRServo clawIntake;
    public final C_Servo airplaneLatch;

    public boolean wristDepositing = false;
    public boolean intakeRollerActive = false;
    public boolean wristTraveling = false;
    public boolean clawDepositing = false;

    public PixelManagement(HardwareMap hardwareMap) {
        intakeRoller = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "intakeRoller"));
        intakeRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRoller.setRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeRoller.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeRoller.toRefresh[6] = true;

        lift = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "lift"));
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.toRefresh[5] = true;

        wristPivot = new C_Servo(hardwareMap.tryGet(Servo.class, "wristPivot"));
        clawPivot = new C_Servo(hardwareMap.tryGet(Servo.class, "clawPivot"));
        clawIntake = new C_CRServo(hardwareMap.tryGet(CRServo.class, "clawIntake"));
        airplaneLatch = new C_Servo(hardwareMap.tryGet(Servo.class, "airplaneLatch"));

        wristPivot.setScaleRange(0.8, 0);
        //DukConstants.AUTOMATED_CONTROLLER_PARAMS.WRIST_PF.FDividend = DukConstants.AUTOMATED_CONTROLLER_PARAMS.WRIST_FULL_ROTATION / (float)(wristPivot.getScaleRange()[0] - wristPivot.getScaleRange()[1]);
        clawPivot.setScaleRange(0.33, 0.85);
        clawIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        airplaneLatch.setScaleRange(1, 0.9);

        dispatchAllCaches();

        clawPivot.setPosition(0.01);
        airplaneLatch.setPosition(0);
    }

    public void dropPixel() {
        //if (!wristTraveling && wristDepositing && !clawDepositing) {
            clawDepositing = true;
            setClawIntakePower(-1);
            TimeManager.scheduleFutureTask(DukConstants.INPUT.CLAW_INTAKE_DROP_TIME, t -> {
                setClawIntakePower(0);
                clawDepositing = false;
                clawIntake.dispatchCache();
            });
        //}
    }

    public void setClawIntakePower(double power) {
        clawIntake.setPower(power * DukConstants.INPUT.CLAW_INTAKE_SPEED);
    }

    public void toggleIntake(boolean reversed) {
        if (intakeRollerActive) {
            intakeRoller.setPower(0);
            clawIntake.setPower(0);
        }
        else {
            clawIntake.setPower(DukConstants.INPUT.CLAW_INTAKE_SPEED);
            intakeRoller.setPower(reversed ? -DukConstants.INPUT.INTAKE_ROLLER_SPEED : DukConstants.INPUT.INTAKE_ROLLER_SPEED);
        }
        intakeRollerActive = !intakeRollerActive;
    }

    public void setIntakeDirection(boolean reversed) {
        if (intakeRollerActive)
            intakeRoller.setPower(reversed ? -DukConstants.INPUT.INTAKE_ROLLER_SPEED : DukConstants.INPUT.INTAKE_ROLLER_SPEED);
    }

    //Hanger is needed to move out of the way
    public void approachLiftTarget(Hanger hanger) {
        lift.setPower(DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PID.evaluate(lift.getCurrentPosition()));
        if ((!wristDepositing && !wristTraveling) || hanger.getTarget() < DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_CLEARANCE_POSITION) return;
        if (lift.getCurrentPosition() < DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_LIFT_DEADZONE)
            hanger.setTarget(0);
        else
            hanger.setTarget(DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_CLEARANCE_POSITION);
    }

    public void setLiftTarget(float target) {
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PID.target = (float)DukUtilities.clamp(
                target,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_MAX,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_MIN);
    }

    public float getLiftTarget() {
        return DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PID.target;
    }

    public void setClawState(boolean deposit) {
        if (wristDepositing == deposit || (wristTraveling && !DukConstants.INPUT.ALLOW_WRIST_CHANGE_WHILE_TRAVEL)) return;
        wristPivot.setTargetPosition(deposit ? 1 : 0);
        wristDepositing = deposit;
        wristTraveling = true;
    }

    public void approachClawState() {
        if (!wristTraveling || (!wristDepositing
                && lift.getCurrentPosition() > DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_HEIGHT_WRIST_REST
                && wristPivot.getPosition() < DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_DESCENT_WRIST_POSITION)) return;
        clawIntake.setPower(DukConstants.INPUT.CLAW_INTAKE_SPEED);
        wristPivot.approachTargetPosition(DukConstants.AUTOMATED_CONTROLLER_PARAMS.WRIST_SPEED);
        if (wristPivot.getPosition() > DukConstants.AUTOMATED_CONTROLLER_PARAMS.WRIST_CLAW_ACTIVATION_THRESHOLD
                && wristPivot.getPosition() < 1 - DukConstants.AUTOMATED_CONTROLLER_PARAMS.WRIST_CLAW_ACTIVATION_THRESHOLD) {
            clawPivot.setPosition(wristDepositing ? 1 : 0);
        }
        if (Math.abs(wristPivot.getPosition() - (wristDepositing ? 1 : 0)) < DukConstants.AUTOMATED_CONTROLLER_PARAMS.WRIST_CUTOFF) {
            wristTraveling = false;
            wristPivot.setPosition(wristDepositing ? 1 : 0);
            clawIntake.setPower(0);
//                    && (!DukConstants.INPUT.HOLD_WRIST_UNTIL_LIFT_DOWN
//                        || lift.getCurrentPosition() < DukConstants.AUTOMATED_CONTROLLER_PARAMS.MAX_WRIST_LIFT_HEIGHT)) {
        }
    }

    public void launchAirplane() {
        airplaneLatch.setPosition(1);
//        TimeManager.scheduleFutureTask(0.5f, t -> airplaneLatch.setPosition(1));
    }

    @Override
    public void dispatchAllCaches() {
        intakeRoller.dispatchCache();
        lift.dispatchCache();
        wristPivot.dispatchCache();
        clawPivot.dispatchCache();
        clawIntake.dispatchCache();
        airplaneLatch.dispatchCache();
    }

    @Override
    public void refreshAllCaches() {
        intakeRoller.refreshCache();
        lift.refreshCache();
        wristPivot.refreshCache();
        clawPivot.refreshCache();
        clawIntake.refreshCache();
        airplaneLatch.refreshCache();
    }

    @Override
    public void pushTelemetry() {
        if (!DukConstants.DEBUG.USE_FTC_DASHBOARD) return;

        loggingBuffer.push("Intake Roller Power", intakeRoller.getPower());
        loggingBuffer.push("Lift Power", lift.getPower());
        loggingBuffer.push("Lift Position", lift.getCurrentPosition());
        loggingBuffer.push("Wrist Pivot", wristPivot.getPosition());
        loggingBuffer.push("Claw Pivot", clawPivot.getPosition());
        loggingBuffer.push("Claw Intake", clawIntake.getPower());
        loggingBuffer.push("Wrist State", wristDepositing);
        loggingBuffer.push("Intake Roller Active", intakeRollerActive);
        loggingBuffer.push("Wrist Traveling", wristTraveling);
        loggingBuffer.push("Ariplane Latch", airplaneLatch.getPosition());
        loggingBuffer.dispatch();
    }

    @Override
    public void allowDispatch(boolean state) {
        intakeRoller.allowDispatch(state);
        lift.allowDispatch(state);
        wristPivot.allowDispatch(state);
        clawPivot.allowDispatch(state);
        clawIntake.allowDispatch(state);
    }
}
