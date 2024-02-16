package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PixelManagement;

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
    public static double lift_p = DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PID.P;
    public static double lift_i = 0;
    public static double lift_d = 0;
    public static double hanger_p = DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_PI.P;
    public static double hanger_i = 0;
    public static double wrist_speed = DukConstants.AUTOMATED_CONTROLLER_PARAMS.WRIST_SPEED;
    public static double intake_speed = DukConstants.INPUT.INTAKE_ROLLER_SPEED  ;
    public static int imageLayer;
    private static int numErrors;

    public static void tick(DukHardwareMap hMap) {
        applyConfig();
        ComponentDispatching.apply(hMap);
        dispatchBufferPacket();
    }

    public static class ComponentDispatching {
        public static void apply(DukHardwareMap hMap) {
            CD_DriveTrain.apply(hMap.driveTrain);
            CD_PixelManagement.apply(hMap.pixelManagement);
            CD_Hanger.apply(hMap.hanger);
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
        @Config
        public static class CD_PixelManagement {
            public static void apply(PixelManagement pixelManagement) {
                pixelManagement.intakeRoller.allowDispatch(intakeRoller);
                pixelManagement.lift.allowDispatch(lift);
                pixelManagement.wristPivot.allowDispatch(wristPivot);
                pixelManagement.clawPivot.allowDispatch(clawPivot);
                pixelManagement.clawIntake.allowDispatch(clawIntake);
                pixelManagement.airplaneLatch.allowDispatch(airplaneLatch);
            }

            public static boolean intakeRoller = true;
            public static boolean lift = true;
            public static boolean wristPivot = true;
            public static boolean clawPivot = true;
            public static boolean clawIntake = true;
            public static boolean airplaneLatch = true;
        }
        @Config
        public static class CD_Hanger {
            public static void apply(Hanger hanger) {
                hanger.hangerMotorL.allowDispatch(hangerMotorL);
                hanger.hangerMotorR.allowDispatch(hangerMotorR);
            }

            public static boolean hangerMotorL;
            public static boolean hangerMotorR;
        }
    }

    public static void applyConfig() {
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.P = (float)rotation_p;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.I = (float)rotation_i;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_ROTATION_PID.D = (float)rotation_d;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.P = (float)pursuit_p;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.I = (float)pursuit_i;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.ROBOT_PURSUIT_PID.D = (float)pursuit_d;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PID.P = (float)lift_p;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PID.I = (float)lift_i;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_PID.D = (float)lift_d;
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_PI.P = (float)(hanger_p);
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.HANGER_PI.I = (float)(hanger_i);
        DukConstants.AUTOMATED_CONTROLLER_PARAMS.WRIST_SPEED = (float)wrist_speed;
        DukConstants.INPUT.INTAKE_ROLLER_SPEED = (float)intake_speed;
        DukConstants.WEBCAM.IMAGE_LAYER = imageLayer;
    }

    public static void renderRobot(String stroke, double posX, double posY, double heading) {
        double posXIn = posX * (1/DukConstants.HARDWARE.ET_PER_INCH);
        double posYIn = posY * (1/DukConstants.HARDWARE.ET_PER_INCH);
        bufferPacket.fieldOverlay()
                .setStroke(stroke)
                .strokeRect(posXIn - DukConstants.HARDWARE.ROBOT_SIZE_IN * 0.5,
                        posYIn - DukConstants.HARDWARE.ROBOT_SIZE_IN * 0.5,
                        DukConstants.HARDWARE.ROBOT_SIZE_IN,
                        DukConstants.HARDWARE.ROBOT_SIZE_IN)
                .strokeLine(posXIn,
                        posYIn,
                        posXIn + Math.sin(heading) * DukConstants.DEBUG.DIRECTION_INDICATOR_LENGTH,
                        posYIn + Math.cos(heading) * DukConstants.DEBUG.DIRECTION_INDICATOR_LENGTH);
    }

    public static void dispatchBufferPacket() {
        dashboard.sendTelemetryPacket(bufferPacket);
        bufferPacket = new TelemetryPacket();
    }

    //Does not buffer
    public static void logError(String label, Object data) {
        if (dashboard == null) return;
        numErrors++;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("E-" + numErrors + ": " + label, data);
        dashboard.sendTelemetryPacket(packet);
    }
}
