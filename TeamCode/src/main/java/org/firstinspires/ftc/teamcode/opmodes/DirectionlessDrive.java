package org.firstinspires.ftc.teamcode.opmodes;
//C:\Users\jerem\AppData\Local\Android\Sdk\platform-tools
//adb connect 192.168.43.1:5555

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.DukUtilities.Vector;
import org.firstinspires.ftc.teamcode.util.PersistentData;
import org.firstinspires.ftc.teamcode.util.TimeManager;

import java.sql.Time;

@TeleOp
public class DirectionlessDrive extends OpMode {
    DukHardwareMap hMap;
    BetterGamepad bGamepad1;
    BetterGamepad bGamepad2;
    boolean blindDrive, blindMechanism;

    @Override
    public void init() {
        hMap = new DukHardwareMap(hardwareMap);
        telemetry.addData("Persistent Available", PersistentData.available);
        PersistentData.Apply(hMap);
        hMap.driveTrain.targetPose = hMap.driveTrain.poseEstimator.getPose();
        hMap.driveTrain.pursueHeading = true;

        bGamepad1 = new BetterGamepad(gamepad1);
        bGamepad2 = new BetterGamepad(gamepad2);
    }

    private void controlChassis() {
        float targetHeading = hMap.driveTrain.targetPose.getH();

        if (DukUtilities.getJoystickMagnitude(gamepad1, true) > DukConstants.INPUT.JOYSTICK_TURN_THRESHOLD)
            targetHeading = DukUtilities.getJoystickDirection(gamepad1, true);
        if (gamepad1.dpad_up) targetHeading = 0;
        if (gamepad1.dpad_down) targetHeading = (float)Math.PI;
        if (gamepad1.dpad_right) targetHeading = (float)Math.PI * 0.5f;
        if (gamepad1.dpad_left) targetHeading = -(float)Math.PI * 0.5f;

        if (gamepad1.right_bumper || gamepad1.left_bumper)
            targetHeading = targetHeading + (gamepad1.right_bumper ? DukConstants.INPUT.MANUAL_TURN_CONTROL_MULTIPLIER : -DukConstants.INPUT.MANUAL_TURN_CONTROL_MULTIPLIER);

        hMap.driveTrain.targetPose.setH(targetHeading);
        hMap.driveTrain.displaceVector(new Vector(gamepad1.right_stick_x, gamepad1.right_stick_y, true), true);
        hMap.driveTrain.forAllMotors(motor -> motor.setPower(motor.getPower()
                + (gamepad1.left_trigger - gamepad1.right_trigger) * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER));
    }

    private void controlChassisBlind() {
        hMap.driveTrain.displaceVector(new Vector(gamepad1.right_stick_x, gamepad1.right_stick_y, true), false);
        hMap.driveTrain.forAllMotors(motor -> motor.setPower(motor.getPower()
                + (gamepad1.right_trigger - gamepad1.left_trigger) * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER));
        hMap.driveTrain.turnRelative(gamepad1.left_stick_x + (gamepad1.right_bumper ? 1 : gamepad1.left_bumper ? -1 : 0));
    }

    private void checkSafetySwitch() {
        if (bGamepad1.onYPressed()) {
            hMap.driveTrain.pursueHeading = blindDrive;
            blindDrive = !blindDrive;
        }
        if (bGamepad1.onXPressed())
            blindMechanism = !blindMechanism;
    }

    private void driveLog() {
        telemetry.addData("Blind Drive", blindDrive);
        telemetry.addData("Blind Mechanism", blindMechanism);
        telemetry.update();
    }

    @Override
    public void loop() {
        checkSafetySwitch();
        hMap.refreshAll();
        if (blindDrive) controlChassisBlind();
        else controlChassis();
        TimeManager.onTick(time);
        hMap.dispatchAll();

        driveLog();
    }

    @Override
    public void stop() {
        TimeManager.reset();
        super.stop();
    }
}