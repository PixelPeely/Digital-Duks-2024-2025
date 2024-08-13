package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;

@Config
public class DashboardInterface {
    //Assigned in DukHardwareMap
    public static FtcDashboard dashboard;
    public static TelemetryPacket bufferPacket = new TelemetryPacket();

    public static double rotation_p = DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.P;
    public static double rotation_i = DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.I;
    public static double rotation_d = DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.D;
    public static double pursuit_p = DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.P;
    public static double pursuit_i = DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.I;
    public static double pursuit_d = DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.D;
    private static int numErrors;

    public static void tick(DukHardwareMap hMap) {
        applyConfig();
        ComponentDispatching.apply(hMap);
        dispatchBufferPacket();
    }

    public static class ComponentDispatching {
        public static void apply(DukHardwareMap hMap) {
            CD_DriveTrain.apply(hMap.driveTrain);
        }
        @Config
        public static class CD_DriveTrain {
            public static void apply(DriveTrain driveTrain) {
                driveTrain.frontLeft.allowDispatch(frontLeft);
                driveTrain.frontRight.allowDispatch(frontRight);
                driveTrain.backLeft.allowDispatch(backLeft);
                driveTrain.backRight.allowDispatch(backRight);
            }

            public static boolean frontLeft = true;
            public static boolean frontRight = true;
            public static boolean backLeft = true;
            public static boolean backRight = true;
        }
    }

    public static void applyConfig() {
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.P = rotation_p;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.I = rotation_i;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.D = rotation_d;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.P = pursuit_p;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.I = pursuit_i;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.D = pursuit_d;
    }

    public static void renderRobot(String stroke, Pose pose) {
        //y=-x, x=y
        Vector pos = DukUtilities.ETToFieldCoords(pose);
        bufferPacket.fieldOverlay()
                .setStroke(stroke)
                .strokeRect(pos.getX() - DukConstants.HARDWARE.ROBOT_SIZE_IN * 0.5,
                        pos.getY() - DukConstants.HARDWARE.ROBOT_SIZE_IN * 0.5,
                        DukConstants.HARDWARE.ROBOT_SIZE_IN,
                        DukConstants.HARDWARE.ROBOT_SIZE_IN)
                .strokeLine(pos.getX(),
                        pos.getY(),
                        pos.getX() + Math.cos(pose.getH()) * DukConstants.DEBUG.DIRECTION_INDICATOR_LENGTH,
                        pos.getY() - Math.sin(pose.getH()) * DukConstants.DEBUG.DIRECTION_INDICATOR_LENGTH);
    }

    public static void dispatchBufferPacket() {
        dashboard.sendTelemetryPacket(bufferPacket);
        bufferPacket = new TelemetryPacket();
    }

    //Does not buffer
    public static void immediateError(String label, Object data) {
        if (dashboard == null) return;
        numErrors++;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("E-" + numErrors + ": " + label, data);
        dashboard.sendTelemetryPacket(packet);
    }
}
