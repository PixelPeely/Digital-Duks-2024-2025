package org.firstinspires.ftc.teamcode.opmodes;
//C:\Users\jerem\AppData\Local\Android\Sdk\platform-tools
//adb connect 192.168.43.1:5555
//./gradlew reloadFastLoad

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shuttle;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SubmersibleIntake;
import org.firstinspires.ftc.teamcode.hardware.wrappers.GamepadExt;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.Vector;

@TeleOp
public class DirectionlessDrive extends DukOpMode {
    boolean blind1, blind2;

    @Override
    public void init() {
        super.init();
        _hardwareMap.driveTrain.pursueHeading = !blind1;
    }

    private void controlDrivetrain(Gamepad gamepad, GamepadExt gamepadExt) {
        double targetHeading = _hardwareMap.driveTrain.targetPose.getH();

        if (gamepadExt.leftJoystick.getR() > DukConstants.INPUT.JOYSTICK_TURN_THRESHOLD)
            targetHeading = gamepadExt.leftJoystick.getT();
//        if (gamepad.dpad_up) targetHeading = 0;
//        if (gamepad.dpad_down) targetHeading = Math.PI;
//        if (gamepad.dpad_right) targetHeading = Math.PI * 0.5f;
//        if (gamepad.dpad_left) targetHeading = -Math.PI * 0.5f;

        if (gamepad.right_bumper || gamepad.left_bumper)
            targetHeading = targetHeading + (gamepad.right_bumper ? DukConstants.INPUT.MANUAL_TURN_CONTROL_MULTIPLIER : -DukConstants.INPUT.MANUAL_TURN_CONTROL_MULTIPLIER);

        _hardwareMap.driveTrain.targetPose.setH(targetHeading);
        _hardwareMap.driveTrain.displaceVector(gamepad1Ext.rightJoystick, true);
        _hardwareMap.driveTrain.targetPose.vel.add(new Vector(0, gamepadExt.getTriggerDifference() * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER));
    }

    private void controlDrivetrainBlind(Gamepad gamepad, GamepadExt gamepadExt) {
        _hardwareMap.driveTrain.displaceVector(gamepadExt.rightJoystick, false);
        _hardwareMap.driveTrain.targetPose.vel.add(new Vector(0, gamepadExt.getTriggerDifference() * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER));
        _hardwareMap.driveTrain.targetPose.w = gamepad.left_stick_x + (gamepad.right_bumper ? 1 : gamepad.left_bumper ? -1 : 0);
    }

    public void controlSubmersibleIntake(Gamepad gamepad, GamepadExt gamepadExt) {
        SubmersibleIntake.STATE state = _hardwareMap.submersibleIntake.getState();

        if (gamepadExt.onAPressed()) {
            switch (state) {
                case SCOUT:
                    _hardwareMap.submersibleIntake.shuttle.setState(
                            _hardwareMap.submersibleIntake.shuttle.getState() == Shuttle.STATE.PICKUP ?
                                    Shuttle.STATE.SCOUT : Shuttle.STATE.PICKUP);
                    break;
                default:
                    _hardwareMap.submersibleIntake.setState(SubmersibleIntake.STATE.SCOUT);
                    break;
            }
        }

        if (gamepadExt.onBPressed()) {
            switch (state) {
                case DROP:
                    _hardwareMap.submersibleIntake.shuttle.claw.toggle();
                    break;
                default:
                    _hardwareMap.submersibleIntake.setState(SubmersibleIntake.STATE.DROP);
                    break;
            }
        }

        if (gamepadExt.onYPressed())
            _hardwareMap.submersibleIntake.setState(SubmersibleIntake.STATE.TRANSFER);

        double rollDelta = (gamepad.dpad_right ? 0 : 0.01) - (gamepad.dpad_left ? 0 : 0.01);
        if (rollDelta > 0) {
            _hardwareMap.submersibleIntake.shuttle.roll.setPosition(
                    DukUtilities.clamp(_hardwareMap.submersibleIntake.shuttle.roll.getPosition() + rollDelta, 1, 0));
            _hardwareMap.submersibleIntake.shuttle.claw.closed = false;
        }
    }

    public void controlLift(Gamepad gamepad, GamepadExt gamepadExt) {
        if (gamepadExt.onDPadDownPressed())
            _hardwareMap.lift.setState(Lift.STATE.LOW_SAMPLE);

        if (gamepadExt.onDPadUpPressed())
            _hardwareMap.lift.setState(Lift.STATE.HIGH_SAMPLE);

        if (gamepadExt.onDPadLeftPressed())
            _hardwareMap.lift.setState(Lift.STATE.LOW_SPECIMEN);

        if (gamepadExt.onDPadRightPressed())
            _hardwareMap.lift.setState(Lift.STATE.HIGH_SPECIMEN);

        if (gamepadExt.onYPressed())
            _hardwareMap.lift.setState(Lift.STATE.TRANSFER);

        if (gamepadExt.onAPressed())
            _hardwareMap.lift.pivotDeposit.claw.toggle();

        _hardwareMap.lift.manualAdjustment(gamepad.right_trigger - gamepad.left_trigger);
    }

    private void checkSafetySwitch() {
//        if (gamepad2Ext.onYPressed()) {
//            _hardwareMap.driveTrain.pursueHeading = blind1;
//            blind1 = !blind1;
//        }
//        if (gamepad1Ext.onXPressed())
//            blind2 = !blind2;
    }

    private void driveLog() {
        telemetry.addData("SubmersibleIntake State", _hardwareMap.submersibleIntake.getState().name());
        telemetry.addData("-Shuttle State", _hardwareMap.submersibleIntake.shuttle.getState().name());
        telemetry.addData("--Shuttle Claw Closed", _hardwareMap.submersibleIntake.shuttle.claw.closed);
        telemetry.addData("Lift State", _hardwareMap.lift.getState().name());
        telemetry.addData("-Pivot State", _hardwareMap.lift.pivotDeposit.getState().name());
        telemetry.addData("--Pivot Claw Closed", _hardwareMap.lift.pivotDeposit.claw.closed);
        telemetry.addData("Blind Drive", blind1);
        telemetry.addData("Blind Mechanism", blind2);
    }

    @Override
    public void preTick() {
        checkSafetySwitch();
        controlDrivetrain(gamepad1, gamepad1Ext);
        controlSubmersibleIntake(gamepad1, gamepad1Ext);
        controlLift(gamepad2, gamepad2Ext);
//        if (blind1) driver1ControlsBlind();
//        else driver1Controls();
//        if (blind2) driver2ControlsBlind();
//        else driver2Controls();
        driveLog();
    }

    @Override
    public void postTick() {

    }
}