package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonPointTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonTaskExecuter;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.TimeManager;

@Autonomous(preselectTeleOp = "DirectionlessDrive")
public class BlueLeftPark extends OpMode {
    private DukHardwareMap hMap;
    AutonTaskExecuter autonTaskExecuter;

    @Override
    public void init() {
        hMap = new DukHardwareMap(hardwareMap);
        hMap.odometerWheels.setBeginningPose(DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.BLUE_LEFT_X,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.BLUE_LEFT_Y,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.BLUE_LEFT_H,
                true);
        autonTaskExecuter = new AutonTaskExecuter();
        autonTaskExecuter.tasks.add(new AutonPointTask(78119, 78670, (float)Math.toRadians(180), 0, 0));
    }

    @Override
    public void loop() {
        TimeManager.onCycle(getRuntime());
        hMap.odometerWheels.refreshAllCaches();

        if (autonTaskExecuter.tick()) {
            autonTaskExecuter.terminate();
            requestOpModeStop();
        }

        if (DukConstants.DEBUG.USE_FTC_DASHBOARD) {
            hMap.pushTelemetryAll();
            autonTaskExecuter.renderPoints();
            DashboardInterface.tick(hMap);
        }

        hMap.driveTrain.dispatchAllCaches();
    }
}
