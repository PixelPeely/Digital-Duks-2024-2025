package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class DriveTrain implements CachedSubsystem {
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(DriveTrain.class.getSimpleName());
    public final C_DcMotor frontLeft;
    public final C_DcMotor frontRight;
    public final C_DcMotor backLeft;
    public final C_DcMotor backRight;

    private float lastPursuedX = 0;
    private float lastPursuedY = 0;
    public float lastRequestedMovementDirection;
    public float lastRequestedTurnDirection;

    public DriveTrain(HardwareMap hardwareMap) {
        frontLeft = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "frontLeft"));
        frontRight = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "frontRight"));
        backLeft = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "backLeft"));
        backRight = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "backRight"));

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        forAllMotors(motor -> {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motor.setRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        });

        dispatchAllCaches();
    }

    public void forAllMotors(Consumer<C_DcMotor> action) {
        action.accept(frontLeft);
        action.accept(frontRight);
        action.accept(backLeft);
        action.accept(backRight);
    }

    public void moveDirectionRelative(double direction) {
        direction += DukConstants.ORIENTATION.DIRECTION_OFFSET;
        double _sin = Math.sin(direction);
        double _cos = Math.cos(direction);
        frontLeft.setPower(_sin);
        backRight.setPower(_sin);
        frontRight.setPower(_cos);
        backLeft.setPower(_cos);
    }

    public void moveDirectionAbsolute(float chassisOrientation, float direction) {
        lastRequestedMovementDirection = direction;
        moveDirectionRelative(direction-chassisOrientation);
    }

    public void pursueAutonPoint(float px, float py, float pAdd, float x, float y, float chassisOrientation) {
        float moveDirection = (float) Math.atan2(px - x, py - y);
        moveDirectionAbsolute(chassisOrientation, moveDirection);
        float distance = DukUtilities.getDistance(x, y, px, py);
        //Inverted because the PIDF will try to approach a distance of zero by returning negative values
        applyMagnitude((float)DukUtilities.clamp(-DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.evaluate(distance + pAdd), 1, -1));
        lastPursuedX = px;
        lastPursuedY = py;
    }

    public void applyMagnitude(float magnitude) {
        if (DukConstants.INPUT.MAXIMIZE_MAGNITUDE) {
            AtomicReference<Double> maxValue = new AtomicReference<>((double)0);
            forAllMotors(motor -> maxValue.set(Math.max(maxValue.get(), Math.abs(motor.getPower()))));
            if (maxValue.get() != 0)
                magnitude /= maxValue.get();
        }
        final double finalMagnitude = magnitude;
        forAllMotors(motor -> motor.setPower(motor.getPower() * finalMagnitude));
    }

    public void turnTowardsDirectionRelative(double directionNormalized) {
        directionNormalized = DukUtilities.clamp(directionNormalized, 1, -1);
        frontLeft.setPower(frontLeft.getPower() - directionNormalized);
        backLeft.setPower(backLeft.getPower() - directionNormalized);
        frontRight.setPower(frontRight.getPower() + directionNormalized);
        backRight.setPower(backRight.getPower() + directionNormalized);
    }

    public void turnTowardsDirectionAbsolute(float chassisOrientation, float direction) {
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.target = direction;
        turnTowardsDirectionRelative(DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.evaluate(chassisOrientation));
        lastRequestedTurnDirection = direction;
    }

    public float getMotorPowerSum() {
        return (float)(frontLeft.getPower() + frontRight.getPower() + backLeft.getPower() + backRight.getPower());
    }

    @Override
    public void dispatchAllCaches() {
        forAllMotors(C_DcMotor::dispatchCache);
    }

    @Override
    public void refreshAllCaches() {
        forAllMotors(C_DcMotor::refreshCache);
    }

    @Override
    public void pushTelemetry() {
        if (!DukConstants.DEBUG.USE_FTC_DASHBOARD) return;

        loggingBuffer.push("Front Left", frontLeft.getPower());
        loggingBuffer.push("Front Right", frontRight.getPower());
        loggingBuffer.push("Back Left", backLeft.getPower());
        loggingBuffer.push("Back Right", backRight.getPower());
        loggingBuffer.push("Pursuit X", lastPursuedX);
        loggingBuffer.push("Pursuit Y", lastPursuedY);
        loggingBuffer.dispatch();

        DashboardInterface.renderRobot(DukConstants.DEBUG.ROBOT_PURSUIT_STROKE,
                lastPursuedX,
                lastPursuedY,
                lastRequestedTurnDirection);
    }

    @Override
    public void allowDispatch(boolean state) {
        forAllMotors(motor -> motor.allowDispatch(state));
    }
}
