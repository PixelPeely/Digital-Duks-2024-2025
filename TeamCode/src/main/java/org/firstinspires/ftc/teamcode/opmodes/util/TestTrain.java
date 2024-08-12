package org.firstinspires.ftc.teamcode.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.DukOpMode;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.DukUtilities.Vector;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp
public class TestTrain extends DukOpMode {
    @Override
    public void init() {
        super.init();
        _hardwareMap.driveTrain.pursueHeading = true;
    }

    @Override
    public void preTick() {
        _hardwareMap.driveTrain.targetPose.x += gamepad1.left_stick_x * 100;
        _hardwareMap.driveTrain.targetPose.y -= gamepad1.left_stick_y * 100;
        _hardwareMap.driveTrain.displaceVector(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y, true), true);
        if (DukUtilities.getJoystickMagnitude(gamepad1, false) > 0.1)
            _hardwareMap.driveTrain.targetPose.setH(DukUtilities.getJoystickDirection(gamepad1, false));
    }

    @Override
    public void postTick() {

    }

    @Override
    public void stop() {
        super.stop();
    }
}
