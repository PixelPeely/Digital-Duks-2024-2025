package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities.Vector;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;
import org.firstinspires.ftc.teamcode.util.TimeManager;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class DriveTrain implements CachedSubsystem {
    public final PoseEstimator poseEstimator;
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer(DriveTrain.class.getSimpleName());
    public final C_DcMotor frontLeft;
    public final C_DcMotor frontRight;
    public final C_DcMotor backLeft;
    public final C_DcMotor backRight;
    public boolean pursueHeading, pursuePosition;
    public Pose targetPose = new Pose();
    private Vector localDisplacement = new Vector(0, 0, true);
    public float localTurning = 0;

    public DriveTrain(HardwareMap hardwareMap) {
        frontLeft = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "frontLeft"));
        frontRight = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "frontRight"));
        backLeft = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "backLeft"));
        backRight = new C_DcMotor(hardwareMap.tryGet(DcMotorEx.class, "backRight"));

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        forAllMotors(motor -> {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motor.setRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        });

        poseEstimator = new PoseEstimator(hardwareMap);
        dispatchAllCaches();

        //TODO consider the final target velocity
        TimeManager.hookTick(t -> {
            if (pursueHeading) {
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.target = targetPose.getH();
                localTurning = DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.evaluate(poseEstimator.getPose().getH());
            }
            if (pursuePosition) {
                float distance = DukUtilities.getDistance(poseEstimator.getPose().x, poseEstimator.getPose().y, targetPose.x, targetPose.y);
                Vector moveVector = new Vector(
                        (float) Math.atan2(targetPose.x - poseEstimator.getPose().x, targetPose.y - poseEstimator.getPose().y),
                        -DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.evaluate(distance),
                        false
                );
                displaceVector(moveVector, true);
            }
            enactDesiredMovement();
            return false;
        });
    }

    public void forAllMotors(Consumer<C_DcMotor> action) {
        action.accept(frontLeft);
        action.accept(frontRight);
        action.accept(backLeft);
        action.accept(backRight);
    }

    private void displaceRelative(double direction) {
        direction += Math.PI * 0.25f;
        double _sin = Math.sin(direction);
        double _cos = Math.cos(direction);
        frontLeft.setPower(_sin);
        backRight.setPower(_sin);
        frontRight.setPower(_cos);
        backLeft.setPower(_cos);
    }

    private void applyMagnitude(float magnitude) {
        if (DukConstants.INPUT.MAXIMIZE_MAGNITUDE) {
            AtomicReference<Double> maxValue = new AtomicReference<>((double)0);
            forAllMotors(motor -> maxValue.set(Math.max(maxValue.get(), Math.abs(motor.getPower()))));
            if (maxValue.get() != 0)
                magnitude /= maxValue.get();
        }
        final double finalMagnitude = magnitude;
        forAllMotors(motor -> motor.setPower(motor.getPower() * finalMagnitude));
    }

    private void turnRelative(double magnitude) {
        magnitude = DukUtilities.clamp(magnitude, 1, -1);
        frontLeft.setPower(frontLeft.getPower() - magnitude);
        backLeft.setPower(backLeft.getPower() - magnitude);
        frontRight.setPower(frontRight.getPower() + magnitude);
        backRight.setPower(backRight.getPower() + magnitude);
    }

    public void displaceVector(Vector displacement, boolean absolute) {
        if (absolute) displacement.rotate(-poseEstimator.getPose().getH());
        localDisplacement = displacement;
    }

    private void enactDesiredMovement() {
        displaceRelative(localDisplacement.getT());
        applyMagnitude(localDisplacement.getR());
        turnRelative(localTurning);
    }

    public void stopMotors() {applyMagnitude(0); dispatchAllCaches();}

    public float getMotorPowerSum() {
        return (float)(frontLeft.getPower() + frontRight.getPower() + backLeft.getPower() + backRight.getPower());
    }

    @Override
    public void dispatchAllCaches() {
        forAllMotors(C_DcMotor::dispatchCache);
        poseEstimator.dispatchAllCaches();
    }

    @Override
    public void refreshAllCaches() {
        forAllMotors(C_DcMotor::refreshCache);
        poseEstimator.refreshAllCaches();
    }

    @Override
    public void pushTelemetry() {
        loggingBuffer.push("Front Left", frontLeft.getPower());
        loggingBuffer.push("Front Right", frontRight.getPower());
        loggingBuffer.push("Back Left", backLeft.getPower());
        loggingBuffer.push("Back Right", backRight.getPower());
        loggingBuffer.push("Pursuit X", targetPose.x);
        loggingBuffer.push("Pursuit Y", targetPose.y);
        loggingBuffer.dispatch();

        DashboardInterface.renderRobot(DukConstants.DEBUG.ROBOT_PURSUIT_STROKE, targetPose);

        poseEstimator.pushTelemetry();
    }

    @Override
    public void allowDispatch(boolean state) {
        forAllMotors(motor -> motor.allowDispatch(state));
        poseEstimator.allowDispatch(state);
    }
}
