package org.firstinspires.ftc.teamcode.opmodes;
//C:\Users\jerem\AppData\Local\Android\Sdk\platform-tools
//adb connect 192.168.43.1:5555
//./gradlew reloadFastLoad

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SubmersibleIntake;
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

    private void driver1Controls() {
        double targetHeading = _hardwareMap.driveTrain.targetPose.getH();

        if (gamepad1Ext.leftJoystick.getR() > DukConstants.INPUT.JOYSTICK_TURN_THRESHOLD)
            targetHeading = gamepad1Ext.leftJoystick.getT();
        if (gamepad1.dpad_up) targetHeading = 0;
        if (gamepad1.dpad_down) targetHeading = Math.PI;
        if (gamepad1.dpad_right) targetHeading = Math.PI * 0.5f;
        if (gamepad1.dpad_left) targetHeading = -Math.PI * 0.5f;

        if (gamepad1.right_bumper || gamepad1.left_bumper)
            targetHeading = targetHeading + (gamepad1.right_bumper ? DukConstants.INPUT.MANUAL_TURN_CONTROL_MULTIPLIER : -DukConstants.INPUT.MANUAL_TURN_CONTROL_MULTIPLIER);

        _hardwareMap.driveTrain.targetPose.setH(targetHeading);
        _hardwareMap.driveTrain.displaceVector(gamepad1Ext.rightJoystick, true);
        _hardwareMap.driveTrain.targetPose.vel.add(new Vector(0, gamepad1Ext.getTriggerDifference() * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER));
    }

    private void driver1ControlsBlind() {
        _hardwareMap.driveTrain.displaceVector(gamepad1Ext.rightJoystick, false);
        _hardwareMap.driveTrain.targetPose.vel.add(new Vector(0, gamepad1Ext.getTriggerDifference() * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER));
        _hardwareMap.driveTrain.targetPose.w = gamepad1.left_stick_x + (gamepad1.right_bumper ? 1 : gamepad1.left_bumper ? -1 : 0);
    }

    public void controlIntake() {
        if (gamepad1Ext.onAPressed()) {
            switch (_hardwareMap.submersibleIntake.getState()) {
                case DROP:
                    _hardwareMap.submersibleIntake.setState(SubmersibleIntake.STATE.SCOUT);
                    break;
                case SCOUT:
                    _hardwareMap.submersibleIntake.setState(SubmersibleIntake.STATE.TRANSFER);
                    break;
            }
        }

        //double shuttleRoll = _hardwareMap.submersibleIntake.shuttle.getRoll();
        //shuttleRoll = DukUtilities.clamp(shuttleRoll + (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0), -Math.PI / 2, Math.PI / 2);
        //_hardwareMap.submersibleIntake.shuttle.setRoll(shuttleRoll);

        if (gamepad1Ext.onBPressed()) _hardwareMap.submersibleIntake.shuttle.claw.setState(!_hardwareMap.submersibleIntake.shuttle.claw.closed);
//
//        if (gamepad2Ext.onAPressed() && _hardwareMap.submersibleIntake.getState() == SubmersibleIntake.STATE.TRANSFER)
//            _hardwareMap.facilitateTransfer();
    }

    public void controlLift() {

    }

    boolean pivot;
    boolean claw;

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
        if (blind1) driver1ControlsBlind();
        else driver1Controls();
        if (blind2) driver2ControlsBlind();
        else driver2Controls();
        driveLog();
    }

    @Override
    public void postTick() {

    }
}