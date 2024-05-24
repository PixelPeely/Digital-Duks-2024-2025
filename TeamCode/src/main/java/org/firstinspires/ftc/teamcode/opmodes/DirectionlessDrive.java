package org.firstinspires.ftc.teamcode.opmodes;
//C:\Users\jerem\AppData\Local\Android\Sdk\platform-tools
//adb connect 192.168.43.1:5555

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.AssessedDiagnostics;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.PersistentData;
import org.firstinspires.ftc.teamcode.util.TimeManager;

@TeleOp
public class DirectionlessDrive extends OpMode {
    DukHardwareMap hMap;
    AssessedDiagnostics assessedDiagnostics;
    BetterGamepad bGamepad1;
    BetterGamepad bGamepad2;
    float targetOrientation = 0;
    boolean blindDrive, blindMechanism;

    @Override
    public void init() {
        hMap = new DukHardwareMap(hardwareMap);
        assessedDiagnostics = new AssessedDiagnostics(hMap);
        telemetry.addData("Persistent Available", PersistentData.available);
        if (PersistentData.available) {
            hMap.odometerWheels.setBeginningPose(PersistentData.positionX, PersistentData.positionY, PersistentData.heading, false);
            telemetry.addData("Position X", PersistentData.positionX);
            telemetry.addData("Position Y", PersistentData.positionY);
            telemetry.addData("Heading", PersistentData.heading);
        }
        hMap.duplexIMU.zeroIMUs(PersistentData.available);
        PersistentData.available = false;
        bGamepad1 = new BetterGamepad(gamepad1);
        bGamepad2 = new BetterGamepad(gamepad2);
    }

    private void controlChassis() {
        float heading = hMap.odometerWheels.getHeading(false);
        if (assessedDiagnostics.odometersFailed) {
            hMap.duplexIMU.refreshAllCaches();
            heading = hMap.duplexIMU.getOrientation().thirdAngle;
        }
        float robotDisplacementDirectionCorrection = (float)Math.atan2(hMap.odometerWheels.positionX, hMap.odometerWheels.positionY);

        if (DukUtilities.getJoystickMagnitude(gamepad1, true) > DukConstants.INPUT.JOYSTICK_TURN_THRESHOLD)
            targetOrientation = DukUtilities.getJoystickDirection(gamepad1, true);
        if (gamepad1.dpad_up) targetOrientation = 0;
        if (gamepad1.dpad_down) targetOrientation = (float)Math.PI;
        if (gamepad1.dpad_right) targetOrientation = (float)Math.PI * 0.5f;
        if (gamepad1.dpad_left) targetOrientation = -(float)Math.PI * 0.5f;
        
        //Should probably come after the move direction call
        if (gamepad1.right_bumper || gamepad1.left_bumper)
            targetOrientation = DukUtilities.constrainAxis(targetOrientation + (gamepad1.right_bumper ? DukConstants.INPUT.MANUAL_TURN_CONTROL_MULTIPLIER : -DukConstants.INPUT.MANUAL_TURN_CONTROL_MULTIPLIER));

        hMap.driveTrain.moveDirectionAbsolute(heading, DukUtilities.getJoystickDirection(gamepad1, false) +
                (DukConstants.ORIENTATION.CORRECT_FOR_DISPLACEMENT_POS ? robotDisplacementDirectionCorrection : 0));
        hMap.driveTrain.applyMagnitude(DukUtilities.getJoystickMagnitude(gamepad1, false));
        hMap.driveTrain.forAllMotors(motor -> motor.setPower(motor.getPower()//Move up if something breaks
                + (gamepad1.left_trigger - gamepad1.right_trigger) * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER));
        hMap.driveTrain.turnTowardsDirectionAbsolute(heading, DukUtilities.constrainAxis(targetOrientation +
                (DukConstants.ORIENTATION.CORRECT_FOR_DISPLACEMENT_ROT ? robotDisplacementDirectionCorrection : 0)));
    }

    private void controlChassisBlind() {
        hMap.driveTrain.moveDirectionRelative(DukUtilities.getJoystickDirection(gamepad1, false));
        hMap.driveTrain.applyMagnitude(DukUtilities.getJoystickMagnitude(gamepad1, false));
        hMap.driveTrain.forAllMotors(motor -> motor.setPower(motor.getPower()
                + (gamepad1.right_trigger - gamepad1.left_trigger) * DukConstants.INPUT.MANUAL_DRIVE_CONTROL_MULTIPLIER));
        hMap.driveTrain.turnTowardsDirectionRelative(gamepad1.left_stick_x
            + (gamepad1.right_bumper ? 1 : gamepad1.left_bumper ? -1 : 0));
    }

    private void checkSafetySwitch() {
        if (bGamepad1.onYPressed())
            blindDrive = !blindDrive;
        if (bGamepad1.onXPressed())
            blindMechanism = !blindMechanism;
    }
    
    @Override
    public void loop() {
        if (DukConstants.DEBUG.USE_FTC_DASHBOARD) {
            hMap.pushTelemetryAll();
            DashboardInterface.tick(hMap);
        }

        TimeManager.onCycle(getRuntime());

        hMap.refreshAll();
        assessedDiagnostics.assessPassive();
        checkSafetySwitch();
        if (blindDrive) controlChassisBlind();
        else controlChassis();
        hMap.dispatchAll();
        telemetry.addData("Blind Drive", blindDrive);
        telemetry.addData("Blind Mechanism", blindMechanism);
        telemetry.update();
    }
}
