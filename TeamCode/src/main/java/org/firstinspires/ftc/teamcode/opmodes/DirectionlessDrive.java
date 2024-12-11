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

@Config
@TeleOp
public class DirectionlessDrive extends DukOpMode {
    boolean blind1, blind2;

    public static boolean a, b, x, y, _a, _b, _x, _y;

    @Override
    public void init() {
        super.init();
        //_hardwareMap.driveTrain.pursueHeading = !blind1;
    }

    private void chassisControls(Gamepad gamepad, GamepadExt gamepadExt) {
        double targetHeading = _hardwareMap.driveTrain.targetPose.getH();

        if (gamepadExt.leftJoystick.getR() > DukConstants.INPUT.JOYSTICK_TURN_THRESHOLD)
            targetHeading = gamepadExt.leftJoystick.getT();
        if (gamepad.dpad_up) targetHeading = 0;
        if (gamepad.dpad_down) targetHeading = Math.PI;
        if (gamepad.dpad_right) targetHeading = Math.PI * 0.5f;
        if (gamepad.dpad_left) targetHeading = -Math.PI * 0.5f;

        if (gamepad.right_bumper || gamepad.left_bumper)
            targetHeading = targetHeading + (gamepad.right_bumper ? DukConstants.INPUT.MANUAL_TURN_CONTROL_MULTIPLIER : -DukConstants.INPUT.MANUAL_TURN_CONTROL_MULTIPLIER);

        _hardwareMap.driveTrain.targetPose.setH(targetHeading);
        _hardwareMap.driveTrain.displaceVector(gamepad1Ext.rightJoystick, true);
        _hardwareMap.driveTrain.targetPose.vel.add(new Vector(0, gamepadExt.getTriggerDifference() * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER));
    }

    private void chassisControlsBlind(Gamepad gamepad, GamepadExt gamepadExt) {
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

        if (gamepadExt.onYPressed()) {
            switch (state) {
                case DROP:
                    _hardwareMap.submersibleIntake.setState(SubmersibleIntake.STATE.TRANSFER);
                    break;
                default:
                    _hardwareMap.submersibleIntake.shuttle.claw.toggle();
            }
        }
    }

    public void controlLift() {

    }

    private void driver2Controls() {

    }

    private void driver2ControlsBlind() {

    }

    private void checkSafetySwitch() {
        if (gamepad1Ext.onYPressed()) {
            _hardwareMap.driveTrain.pursueHeading = blind1;
            blind1 = !blind1;
        }
        if (gamepad1Ext.onXPressed())
            blind2 = !blind2;
    }

    private void driveLog() {
        telemetry.addData("Blind Drive", blind1);
        telemetry.addData("Blind Mechanism", blind2);
    }

    @Override
    public void preTick() {
        checkSafetySwitch();
        controlSubmersibleIntake(gamepad1, gamepad1Ext);
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