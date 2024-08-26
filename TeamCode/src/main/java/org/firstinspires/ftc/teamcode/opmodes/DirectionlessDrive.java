package org.firstinspires.ftc.teamcode.opmodes;
//C:\Users\jerem\AppData\Local\Android\Sdk\platform-tools
//adb connect 192.168.43.1:5555

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.util.PersistentData;
import org.firstinspires.ftc.teamcode.util.TimeManager;

@TeleOp
public class DirectionlessDrive extends DukOpMode {
    boolean blindDrive, blindMechanism;

    @Override
    public void init() {
        super.init();
        _hardwareMap.driveTrain.pursueHeading = true;
    }

    private void controlChassis() {
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
        _hardwareMap.driveTrain.targetPose.vel.add(new Vector(gamepad1Ext.getTriggerDifference() * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER, 0));
    }

    private void controlChassisBlind() {
        _hardwareMap.driveTrain.displaceVector(gamepad1Ext.rightJoystick, false);
        _hardwareMap.driveTrain.targetPose.vel.add(new Vector(gamepad1Ext.getTriggerDifference() * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER, 0));
        _hardwareMap.driveTrain.targetPose.w = gamepad1.left_stick_x + (gamepad1.right_bumper ? 1 : gamepad1.left_bumper ? -1 : 0);
    }

    private void checkSafetySwitch() {
        if (gamepad1Ext.onYPressed()) {
            _hardwareMap.driveTrain.pursueHeading = blindDrive;
            blindDrive = !blindDrive;
        }
        if (gamepad1Ext.onXPressed())
            blindMechanism = !blindMechanism;
    }

    private void driveLog() {
        telemetry.addData("Blind Drive", blindDrive);
        telemetry.addData("Blind Mechanism", blindMechanism);
    }

    @Override
    public void preTick() {
        checkSafetySwitch();
        if (blindDrive) controlChassisBlind();
        else controlChassis();
        driveLog();
    }

    @Override
    public void postTick() {

    }
}