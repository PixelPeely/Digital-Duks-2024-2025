package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DukEye;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DuplexIMU;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.hardware.subsystems.OdometerWheels;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PixelManagement;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;

public class DukHardwareMap {
    public static DukHardwareMap instance = null;

    public DriveTrain driveTrain;
    public DuplexIMU duplexIMU;
    public OdometerWheels odometerWheels;
    public PixelManagement pixelManagement;
    public Hanger hanger;
    public DukEye dukEye;

    public DukHardwareMap(HardwareMap hardwareMap) {
        instance = this;
        if (DukConstants.DEBUG.USE_FTC_DASHBOARD) {
            DashboardInterface.dashboard = FtcDashboard.getInstance();
            DashboardInterface.dashboard.setTelemetryTransmissionInterval(DukConstants.DEBUG.PACKET_TRANSMISSION_INTERVAL);
        }

        driveTrain = new DriveTrain(hardwareMap);
        duplexIMU = new DuplexIMU(hardwareMap);
        odometerWheels = new OdometerWheels(hardwareMap);
        pixelManagement = new PixelManagement(hardwareMap);
        hanger = new Hanger(hardwareMap);
        dukEye = new DukEye(hardwareMap);
    }

    public void dispatchAll() {
        driveTrain.dispatchAllCaches();
        pixelManagement.dispatchAllCaches();
        hanger.dispatchAllCaches();
    }

    public void refreshAll() {
        odometerWheels.refreshAllCaches();
        pixelManagement.refreshAllCaches();
        hanger.refreshAllCaches();
    }

    public void pushTelemetryAll() {
        driveTrain.pushTelemetry();
        duplexIMU.pushTelemetry();
        odometerWheels.pushTelemetry();
        pixelManagement.pushTelemetry();
        hanger.pushTelemetry();
    }
}
