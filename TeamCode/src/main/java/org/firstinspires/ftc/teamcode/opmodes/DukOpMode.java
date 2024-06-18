package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.GamepadExt;
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
        telemetry.addData("Persistent Available", PersistentData.available);
        PersistentData.apply(_hardwareMap);
        _hardwareMap.driveTrain.targetPose = _hardwareMap.driveTrain.poseEstimator.getPose();
        setup();
    }

    @Override
    public void loop() {
        if (!TimeManager.hasFirstTickRun()) start();
        _hardwareMap.refreshAll();
        preTick();
        TimeManager.onTick(time);
        postTick();
        _hardwareMap.dispatchAll();
        _hardwareMap.pushTelemetryAll();
        telemetry.addData("OpMode Loop Time (ms)", TimeManager.getDeltaTime() * 1000);
    }

    @Override
    public void stop() {
        TimeManager.reset();
        _hardwareMap.driveTrain.stopMotors();
        super.stop();
    }


    /**
     * Called when "init" is pressed on the driver station after the OpMode has been set up
     */
    public abstract void setup();

    /**
     * Called only once during the first loop cycle when the program starts (before refreshing hardware)
     */
    public abstract void start();

    /**
     * Called every loop before TimeManager is ticked
     */
    public abstract void preTick();

    /**
     * Called every loop after TimeManager is ticked
     */
    public abstract void postTick();
}
