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

    private void controlMechanism() {
        //Intake
        if (bGamepad2.onXPressed())
            hMap.pixelManagement.toggleIntake(false);
        hMap.pixelManagement.setIntakeDirection(gamepad2.b);

        //Lift
        if (gamepad2.dpad_down) hMap.pixelManagement.setLiftTarget(DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_MIN);
        else if (gamepad2.dpad_right) hMap.pixelManagement.setLiftTarget(DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PRESET_1);
        else if (gamepad2.dpad_left) hMap.pixelManagement.setLiftTarget(DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PRESET_2);
        else if (gamepad2.dpad_up) hMap.pixelManagement.setLiftTarget(DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PRESET_3);
        else hMap.pixelManagement.setLiftTarget(hMap.pixelManagement.getLiftTarget() + (gamepad2.right_trigger - gamepad2.left_trigger) * DukConstants.INPUT.LIFT_INCREMENT);
        hMap.pixelManagement.approachLiftTarget(hMap.hanger);

        //Claw
        if (bGamepad2.onYPressed())
            hMap.pixelManagement.setClawState(!hMap.pixelManagement.wristDepositing);
        hMap.pixelManagement.approachClawState();
        if (bGamepad2.onAPressed() && hMap.pixelManagement.wristDepositing)
            hMap.pixelManagement.dropPixel();

        //Hanger
        if (gamepad2.left_bumper || gamepad2.right_bumper)
            hMap.hanger.setTarget(hMap.hanger.getAveragePosition() + DukConstants.INPUT.HANGER_INCREMENT *
            (gamepad2.right_bumper ? 1 : gamepad2.left_bumper ? -1 : 0));
        hMap.hanger.approachTarget();

        //Airplane
        if (bGamepad2.onBackPressed()) hMap.pixelManagement.launchAirplane();
        if (gamepad2.left_stick_button) hMap.hanger.setTarget(DukConstants.INPUT.HANGER_AIRPLANE_POSITION);
    }

    private void controlMechanismBlind() {
        //Intake
        if (bGamepad2.onXPressed())
            hMap.pixelManagement.toggleIntake(gamepad2.b);

        //Lift
        hMap.pixelManagement.lift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        //Claw
        if (bGamepad2.onYPressed())
            hMap.pixelManagement.setClawState(!hMap.pixelManagement.wristDepositing);
        hMap.pixelManagement.approachClawState();
        if (hMap.pixelManagement.wristDepositing)
            hMap.pixelManagement.setClawIntakePower(gamepad2.a ? DukConstants.INPUT.CLAW_INTAKE_SPEED : 0);

        //Hanger
        hMap.hanger.setPower((float) ((gamepad2.left_bumper ? 0.6 : 0) - (gamepad2.right_bumper ? 0.6 : 0)));

        //Airplane
        if (bGamepad2.onBackPressed()) hMap.pixelManagement.launchAirplane();
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

        if (!TimeManager.hasFirstCycleRun()) hMap.pixelManagement.dispatchAllCaches();
        TimeManager.onCycle(getRuntime());

        hMap.refreshAll();
        assessedDiagnostics.assessPassive();
        checkSafetySwitch();
        if (blindDrive) controlChassisBlind();
        else controlChassis();
        if (blindMechanism) controlMechanismBlind();
        else controlMechanism();
        hMap.dispatchAll();
        telemetry.addData("Blind Drive", blindDrive);
        telemetry.addData("Blind Mechanism", blindMechanism);
        telemetry.update();
    }
}
