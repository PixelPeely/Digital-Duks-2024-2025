package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.AssessedDiagnostics;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.TimeManager;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonClawTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonConditionalPointTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonIntakeTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonPointTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonTaskExecuter;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonWristTask;

@Autonomous(preselectTeleOp="DirectionlessDrive")
public class BlueLeftSpike extends OpMode {
    DukHardwareMap hMap;
    AutonTaskExecuter autonTaskExecuter;
    AssessedDiagnostics assessedDiagnostics;

    @Override
    public void init() {
        hMap = new DukHardwareMap(hardwareMap);
        TimeManager.reset();
        hMap.odometerWheels.setBeginningPose(DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.BLUE_LEFT_X,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.BLUE_LEFT_Y,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.BLUE_LEFT_H,
                true);
        assessedDiagnostics = new AssessedDiagnostics(hMap);
        autonTaskExecuter = new AutonTaskExecuter();
        autonTaskExecuter.tasks.add(new AutonPointTask(16000, 60000, (float)Math.toRadians(180), 0, 0));
        autonTaskExecuter.tasks.add(new AutonConditionalPointTask(() ->{
            switch (hMap.dukEye.spikeIndex) {
                case 1:
                    return new AutonPointTask(40000,  40400,
                        (float)Math.toRadians(-90), 0, 0);
                case 2:
                    return new AutonPointTask(25000, 32000,
                        (float)Math.toRadians(-90), 0, 0);
            }
            return new AutonPointTask(9500, 44700,
                    (float)Math.toRadians(-90), 0, 0);
        })); //Defaults to 3 if no spike detected
        autonTaskExecuter.tasks.add(new AutonIntakeTask(true, 1.2f));
        autonTaskExecuter.tasks.add(new AutonPointTask(51300, 48400, (float)Math.toRadians(-90), 0, 0));
        autonTaskExecuter.tasks.add(new AutonWristTask(true));
        autonTaskExecuter.tasks.add(new AutonConditionalPointTask(() -> {
            switch (hMap.dukEye.spikeIndex) {
                case 1:
                    return new AutonPointTask(65000, 57000, (float)Math.toRadians(-90), 0, 0);
                case 2:
                    return new AutonPointTask(65000, 50000, (float)Math.toRadians(-90), 0, 0);
            }
            return new AutonPointTask(65000, 36700, (float)Math.toRadians(-90), 0, 0);
        }));
        autonTaskExecuter.tasks.add(new AutonClawTask(2));
        autonTaskExecuter.tasks.add(new AutonWristTask(false));
        autonTaskExecuter.tasks.add(new AutonPointTask(50000, 48100, (float)Math.toRadians(-90), 0, 0));
        autonTaskExecuter.tasks.add(new AutonPointTask(50000, 15300, (float)Math.toRadians(-90), 0, 0));
        autonTaskExecuter.tasks.add(new AutonPointTask(80000, 15300, (float)Math.toRadians(-90), 0, 0));
    }

    private void onFirstCycle() {
        hMap.duplexIMU.zeroIMUs(false);
    }

    @Override
    public void loop() {
        if (!TimeManager.hasFirstCycleRun()) onFirstCycle();
        TimeManager.onCycle(getRuntime());
        if (hMap.dukEye.spikeIndex == 0 && TimeManager.getTime(false) < 10) {
            hMap.dukEye.observeSpikeMark(false);
            return;
        }
        hMap.refreshAll();
        System.out.println((hMap.dukEye.spikeIndex));
        if (autonTaskExecuter.tick()) {
            autonTaskExecuter.terminate();
            requestOpModeStop();
        }

        if (DukConstants.DEBUG.USE_FTC_DASHBOARD) {
            hMap.pushTelemetryAll();
            autonTaskExecuter.renderPoints();
            DashboardInterface.tick(hMap);
        }
        assessedDiagnostics.assessPassive();
        hMap.dispatchAll();
    }
}