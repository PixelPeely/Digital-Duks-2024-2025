package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.AssessedDiagnostics;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.PersistentData;
import org.firstinspires.ftc.teamcode.util.TimeManager;

@Config
@Autonomous(name="UTIL FreePursuitTest")
public class FreePursuitTest extends OpMode {
    DukHardwareMap hMap;
    public static double testX, testY, testH;
    public static double testThetaBegin, testThetaEnd;
    public static double speed;
    public static double tolerance;
    private final C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer("Auton");
    private AssessedDiagnostics assessedDiagnostics;
    private boolean running = false;
    private BetterGamepad bGamepad1;

    @Override
    public void init() {
        TimeManager.reset();
        hMap = new DukHardwareMap(hardwareMap);
        hMap.odometerWheels.setBeginningPose(DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.RED_LEFT_X,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.RED_LEFT_Y,
                DukConstants.AUTOMATED_CONTROLLER_PARAMS.AUTON_BEGIN_POSES.RED_RIGHT_H, true);
        assessedDiagnostics = new AssessedDiagnostics(hMap);
        bGamepad1 = new BetterGamepad(gamepad1);
    }

    void onFirstCycle() {
        hMap.duplexIMU.zeroIMUs(false);
        DashboardInterface.logError("Triggered", 0);
    }

    @Override
    public void loop() {
        if (bGamepad1.onAPressed())
            running = !running;
        DashboardInterface.applyConfig();
        DashboardInterface.ComponentDispatching.apply(hMap);
        if (!TimeManager.hasFirstCycleRun()) onFirstCycle();
        TimeManager.onCycle(getRuntime());
        testX -= gamepad1.left_stick_y * speed;
        testY -= gamepad1.left_stick_x * speed;

        if (DukUtilities.getJoystickMagnitude(gamepad1, false) > 0.5f)
            testH = DukUtilities.constrainAxis(DukUtilities.getJoystickDirection(gamepad1, false) + (float) Math.PI * 0.5f);
        hMap.driveTrain.pursueAutonPoint((float) testX, (float) testY, 0,
                hMap.odometerWheels.positionX, hMap.odometerWheels.positionY,
                hMap.odometerWheels.getHeading(true));
        hMap.driveTrain.turnTowardsDirectionAbsolute(hMap.odometerWheels.getHeading(true), (float) testH);

        if (!running)
            hMap.driveTrain.applyMagnitude(0);

        hMap.odometerWheels.refreshAllCaches();
       // assessedDiagnostics.assessPassive();
        hMap.pushTelemetryAll();
        hMap.driveTrain.dispatchAllCaches();
        DashboardInterface.dispatchBufferPacket();
    }

    @Override
    public void stop() {
        PersistentData.available = true;
        PersistentData.positionX = hMap.odometerWheels.positionX;
        PersistentData.positionY = hMap.odometerWheels.positionY;
        PersistentData.heading = hMap.odometerWheels.getHeading(true);
    }
}
