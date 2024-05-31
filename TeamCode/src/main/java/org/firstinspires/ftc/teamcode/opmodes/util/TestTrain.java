package org.firstinspires.ftc.teamcode.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.DukUtilities.Vector;
import org.firstinspires.ftc.teamcode.util.TimeManager;

@TeleOp
public class TestTrain extends OpMode {
    DukHardwareMap hMap;

    @Override
    public void init() {
        hMap = new DukHardwareMap(hardwareMap);
        hMap.driveTrain.pursueHeading = true;
    }

    @Override
    public void loop() {
        hMap.driveTrain.refreshAllCaches();

        hMap.driveTrain.targetPose.x += gamepad1.left_stick_x * 100;
        hMap.driveTrain.targetPose.y -= gamepad1.left_stick_y * 100;
        hMap.driveTrain.displaceVector(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y, true), true);
        if (DukUtilities.getJoystickMagnitude(gamepad1, false) > 0.1)
            hMap.driveTrain.targetPose.setH(DukUtilities.getJoystickDirection(gamepad1, false));

        telemetry.addData("Tick Delta (ms)", TimeManager.getDeltaTime() * 1000);
        TimeManager.onTick(time);
        hMap.driveTrain.dispatchAllCaches();
    }

    @Override
    public void stop() {
        TimeManager.reset();
        super.stop();
    }
}
