package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.hardware.wrappers.GamepadExt;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.PersistentData;
import org.firstinspires.ftc.teamcode.util.TimeManager;

public abstract class DukOpMode extends OpMode {
    public DukHardwareMap _hardwareMap;
    public C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer("OpMode");
    public GamepadExt gamepad1Ext;
    public GamepadExt gamepad2Ext;

    @Override
    public void init() {
        gamepad1Ext = new GamepadExt(gamepad1);
        gamepad2Ext = new GamepadExt(gamepad2);
        _hardwareMap = new DukHardwareMap(hardwareMap);
        _hardwareMap.allHubs.forEach(h -> h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        telemetry.addData("Persistent Available", PersistentData.available);
        PersistentData.apply(_hardwareMap);
        _hardwareMap.driveTrain.targetPose = _hardwareMap.driveTrain.poseEstimator.getPose();
    }

    @Override
    public void start() {
        TimeManager.onTick(time);
        _hardwareMap.dispatchAll();
    }

    @Override
    public void loop() {
        _hardwareMap.allHubs.forEach(LynxModule::clearBulkCache);
        _hardwareMap.refreshAll();
        preTick();
        TimeManager.onTick(time);
        postTick();
        _hardwareMap.dispatchAll();
        telemetry.addData("OpMode Loop Time (ms)", TimeManager.getDeltaTime() * 1000);
        telemetry.update();
    }

    @Override
    public void stop() {
        TimeManager.reset();
        _hardwareMap.driveTrain.stopMotors();
        Logger.writeLog(!PersistentData.available);//Will only be available if auto has run
        PersistentData.available = false;
        super.stop();
    }

    /**
     * Called every loop before TimeManager is ticked
     */
    public abstract void preTick();

    /**
     * Called every loop after TimeManager is ticked
     */
    public abstract void postTick();
}
