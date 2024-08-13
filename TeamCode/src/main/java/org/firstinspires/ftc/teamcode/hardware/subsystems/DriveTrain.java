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
import org.firstinspires.ftc.teamcode.util.Vector;
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
            motor.dispatchCache();
        });

        poseEstimator = new PoseEstimator(hardwareMap);

        //TODO consider the final target velocity
        TimeManager.hookTick(t -> {
            if (pursueHeading) {
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.target = targetPose.getH();
                targetPose.w = DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.evaluate(poseEstimator.getPose().getH());
            }
            if (pursuePosition) {
                Vector relativeTargetPos = new Vector(targetPose.pos);
                relativeTargetPos.subtract(poseEstimator.getPose().pos);
                relativeTargetPos.scale(-DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.evaluate(relativeTargetPos.getR()));
                displaceVector(relativeTargetPos, true);
            }
            enactTargetVelocity();
            return false;
        });
    }

    public void forAllMotors(Consumer<C_DcMotor> action) {
        action.accept(frontLeft);
        action.accept(frontRight);
        action.accept(backLeft);
        action.accept(backRight);
    }

    private void enactTargetVelocity() {
//        double leftDot = DukConstants.HARDWARE.LEFT_WHEEL_PAIR_PROFILE.dot(targetPose.vel);
//        double rightDot = DukConstants.HARDWARE.RIGHT_WHEEL_PAIR_PROFILE.dot(targetPose.vel);
//
//        frontLeft.setPower(leftDot + targetPose.w);
//        frontRight.setPower(rightDot - targetPose.w);
//        backLeft.setPower(rightDot + targetPose.w);
//        backRight.setPower(leftDot - targetPose.w);

        Vector frontLeftTurn = new Vector(DukConstants.HARDWARE.LEFT_WHEEL_PAIR_PROFILE);
        Vector frontRightTurn = new Vector(DukConstants.HARDWARE.RIGHT_WHEEL_PAIR_PROFILE);
        frontLeftTurn.scale(targetPose.w);
        frontRightTurn.scale(targetPose.w);
        Vector backLeftTurn = new Vector(frontRightTurn);
        Vector backRightTurn = new Vector(frontLeftTurn);
        frontRightTurn.negate();
        backRightTurn.negate();

        frontLeftTurn.add(targetPose.vel);
        frontRightTurn.add(targetPose.vel);
        backLeftTurn.add(targetPose.vel);
        backRightTurn.add(targetPose.vel);

        frontLeft.setPower(frontLeftTurn.dot(DukConstants.HARDWARE.LEFT_WHEEL_PAIR_PROFILE));
        frontRight.setPower(frontRightTurn.dot(DukConstants.HARDWARE.RIGHT_WHEEL_PAIR_PROFILE));
        backLeft.setPower(backLeftTurn.dot(DukConstants.HARDWARE.RIGHT_WHEEL_PAIR_PROFILE));
        backRight.setPower(backRightTurn.dot(DukConstants.HARDWARE.LEFT_WHEEL_PAIR_PROFILE));

        normalizeMotors();
    }

    private void normalizeMotors() {
        AtomicReference<Double> maxValue = new AtomicReference<>((double)0);
        forAllMotors(motor -> maxValue.set(Math.max(maxValue.get(), Math.abs(motor.getPower()))));
        if (maxValue.get() > 1) {
            final double multiple = 1 / maxValue.get();
            forAllMotors(motor -> motor.setPower(motor.getPower() * multiple));
        }
    }

    public void displaceVector(Vector displacement, boolean fieldCentric) {
        Vector relativeDisplacement = new Vector(displacement);
        if (fieldCentric) relativeDisplacement.rotate(-poseEstimator.getPose().getH());
        targetPose.vel = relativeDisplacement;
    }

    public void stopMotors() {forAllMotors(motor -> motor.setPower(0)); dispatchAllCaches();}

    @Override
    public void dispatchAllCaches() {
        forAllMotors(C_DcMotor::dispatchCache);
        poseEstimator.dispatchAllCaches();
    }

    @Override
    public void refreshAllCaches() {
        //forAllMotors(C_DcMotor::refreshCache);
        poseEstimator.refreshAllCaches();
    }

    @Override
    public void pushTelemetry() {
        loggingBuffer.push("Front Left", frontLeft.getPower());
        loggingBuffer.push("Front Right", frontRight.getPower());
        loggingBuffer.push("Back Left", backLeft.getPower());
        loggingBuffer.push("Back Right", backRight.getPower());
        loggingBuffer.push("Velocity X", targetPose.vel.getX());
        loggingBuffer.push("Velocity Y", targetPose.vel.getY());
        loggingBuffer.push("Pursuit X", targetPose.pos.getX());
        loggingBuffer.push("Pursuit Y", targetPose.pos.getY());
        loggingBuffer.push("Velocity W", targetPose.w);
        loggingBuffer.push("Pursuit H", targetPose.getH());
        loggingBuffer.dispatch();

        DashboardInterface.renderRobot(DukConstants.DEBUG.STROKES.ROBOT_PURSUIT_STROKE, targetPose);

        poseEstimator.pushTelemetry();
    }

    @Override
    public void allowDispatch(boolean state) {
        forAllMotors(motor -> motor.allowDispatch(state));
        poseEstimator.allowDispatch(state);
    }
}
