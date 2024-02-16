package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;

@Config
@TeleOp(name="UTIL FieldPositioningUtility")
public class FieldPositioningUtility extends OpMode {
    DukHardwareMap hMap;
    C_TelemetryLoggingBuffer loggingBuffer = new C_TelemetryLoggingBuffer("Test");
    float x, y;
    public static double speed = 1000;

    @Override
    public void init() {
        hMap = new DukHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
        x -= gamepad1.left_stick_x * speed;
        y -= gamepad1.left_stick_y * speed;
        DashboardInterface.renderRobot(DukConstants.DEBUG.ROBOT_PURSUIT_STROKE,
                x, y,
                DukUtilities.constrainAxis(DukUtilities.getJoystickDirection(gamepad1, false) + (float)Math.PI*0.5f));
        DashboardInterface.dispatchBufferPacket();
        loggingBuffer.push("X", x);
        loggingBuffer.push("Y", y);
        loggingBuffer.dispatch();
    }
}
