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
import org.firstinspires.ftc.teamcode.util.autonomous.AutonWaitTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonWristTask;

@Autonomous(preselectTeleOp="DirectionlessDrive")
public class RedLeftSpike extends OpMode {
    DukHardwareMap hMap;
    AutonTaskExecuter autonTaskExecuter;
    AssessedDiagnostics assessedDiagnostics;

    @Override
    public void init() {
        hMap = new DukHardwareMap(hardwareMap);
        TimeManager.reset();
        hMap.odometerWheels.setBeginningPose(DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.RED_LEFT_X,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.RED_LEFT_Y,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.RED_LEFT_H,
                true);
        assessedDiagnostics = new AssessedDiagnostics(hMap);
        autonTaskExecuter = new AutonTaskExecuter();
        autonTaskExecuter.tasks.add(new AutonPointTask(-46900, -61800, 0, 0, 0));
        autonTaskExecuter.tasks.add(new AutonConditionalPointTask(() ->{
            switch (hMap.dukEye.spikeIndex) {
                case 1:
                    return new AutonPointTask(-70900,  -39600,
                        (float)Math.toRadians(90), 0, 0);
                case 2:
                    return new AutonPointTask(-57000, -31000,
                            (float)Math.toRadians(90), 0, 0);
            }
            return new AutonPointTask(-44000, -44500,
                    (float)Math.toRadians(90), 0, 0);
        })); //Defaults to 3 if no spike detected
        autonTaskExecuter.tasks.add(new AutonIntakeTask(true, 1.2f));
        autonTaskExecuter.tasks.add(new AutonPointTask(-75100, -45600, (float)Math.toRadians(0), 0, 0));
        autonTaskExecuter.tasks.add(new AutonPointTask(-75100, -12000, (float)Math.toRadians(0), 0, 0));
        autonTaskExecuter.tasks.add(new AutonPointTask(-65100, -12000, (float)Math.toRadians(-90), 0, 0));
        autonTaskExecuter.tasks.add(new AutonWaitTask(15));
        autonTaskExecuter.tasks.add(new AutonPointTask(45000, -12000, (float)Math.toRadians(-90), 0, 0));
        autonTaskExecuter.tasks.add(new AutonPointTask(45000, -51000, (float)Math.toRadians(-90), 0, 0));
        autonTaskExecuter.tasks.add(new AutonWristTask(true));
        autonTaskExecuter.tasks.add(new AutonConditionalPointTask(() -> {
            switch (hMap.dukEye.spikeIndex) {
                case 1:
                    return new AutonPointTask(65000, -57000, (float)Math.toRadians(-90), 0, 0);
                case 2:
                    return new AutonPointTask(65000, -49000, (float)Math.toRadians(-90), 0, 0);
            }
            return new AutonPointTask(65000, -38500, (float)Math.toRadians(-90), 0, 0);
        }));
        autonTaskExecuter.tasks.add(new AutonClawTask(2));
        autonTaskExecuter.tasks.add(new AutonWristTask(false));
//        autonTaskExecuter.tasks.add(new AutonPointTask(60000, -48100, (float)Math.toRadians(-90), 0, 0));
//        autonTaskExecuter.tasks.add(new AutonPointTask(60000, -81000, (float)Math.toRadians(-90), 0, 0));
//        autonTaskExecuter.tasks.add(new AutonPointTask(80000, -81000, (float)Math.toRadians(-90), 0, 0));
    }

    private void onFirstCycle() {
        hMap.duplexIMU.zeroIMUs(false);
    }

    @Override
    public void loop() {
        if (!TimeManager.hasFirstCycleRun()) onFirstCycle();
        TimeManager.onCycle(getRuntime());
        if (hMap.dukEye.spikeIndex == 0 && TimeManager.getTime(false) < 10) {
            hMap.dukEye.observeSpikeMark(true);
            return;
        }
        hMap.refreshAll();
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

    @Override
    public void stop() {
        autonTaskExecuter.terminate();
    }
}