package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.AssessedDiagnostics;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.PersistentData;
import org.firstinspires.ftc.teamcode.util.TimeManager;

@TeleOp(name="UTIL Test")
@Config
public class test extends OpMode {
    DukHardwareMap hMap;
    AssessedDiagnostics assessedDiagnostics;
    public static double pixelI;
    public static double pixelO;
    public static double wrist;
    public static double liftSpeed;
    public static double intakeRoller;
    public static double heading;
    public static double positionX, positionY;
    public static double speed;
    public static double direction;
    IMU imu;

    @Override
    public void init() {
        telemetry.addData("Version", 6);
        telemetry.addData("Persistent", PersistentData.available);
        telemetry.update();
        hMap = new DukHardwareMap(hardwareMap);
        assessedDiagnostics = new AssessedDiagnostics(hMap);
    }

    private void onFirstCycle() {
        //hMap.pixelManagement.wristPivot.setScaleRange(0, 1);
        //hMap.pixelManagement.dispatchAllCaches();
    }

    @Override
    public void loop() {
        if (!TimeManager.hasFirstCycleRun()) onFirstCycle();
        TimeManager.onCycle(time);
        DashboardInterface.tick(hMap);

        hMap.pixelManagement.airplaneLatch.setPosition(speed);
        hMap.pixelManagement.airplaneLatch.dispatchCache();

        DashboardInterface.dispatchBufferPacket();
    }
}
