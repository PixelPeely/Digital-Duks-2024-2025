package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="UTIL FieldPositioningUtility")
public class FieldPositioningUtility extends OpMode {
    DukHardwareMap hMap;
    C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer("Test");
    Pose pose = new Pose();

    public static double speed = 1000;
    private static final List<Pose> poses = new ArrayList<>();
    BetterGamepad bGamepad1 = new BetterGamepad(gamepad1);

    @Override
    public void init() {
        hMap = new DukHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
        pose.x += gamepad1.left_stick_x * speed;
        pose.y -= gamepad1.left_stick_y * speed;
        if (DukUtilities.getJoystickMagnitude(gamepad1, false) > 0.1)
            pose.setH(DukUtilities.getJoystickDirection(gamepad1, false));
        DashboardInterface.renderRobot(DukConstants.DEBUG.ROBOT_POSE_STROKE, pose);

        if (bGamepad1.onAPressed())
            poses.add(pose);
        else if (bGamepad1.onBPressed() && !poses.isEmpty())
            poses.remove(poses.size() - 1);

        loggingBuffer.push("X", pose.x);
        loggingBuffer.push("Y", pose.y);
        loggingBuffer.push("H", pose.getH());
        loggingBuffer.dispatch();
    }
}
