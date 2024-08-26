package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.TimeManager;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonTask;

import java.util.List;

public class DukHardwareMap {
    public List<LynxModule> allHubs;
    public DriveTrain driveTrain;

    public DukHardwareMap(HardwareMap hardwareMap) {
        if (DukConstants.DEBUG.USE_FTC_DASHBOARD) {
            DashboardInterface.dashboard = FtcDashboard.getInstance();
            DashboardInterface.dashboard.setTelemetryTransmissionInterval(DukConstants.DEBUG.PACKET_TRANSMISSION_INTERVAL);
            TimeManager.hookTick(t -> {
                pushTelemetryAll();
                DashboardInterface.tick(this);
                return false;
            });
        }

        driveTrain = new DriveTrain(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        AutonTask.Base._hardwareMap = this;
    }

    public void dispatchAll() {
        driveTrain.dispatchAllCaches();
    }

    public void refreshAll() {
        driveTrain.refreshAllCaches();
    }

    public void pushTelemetryAll() {
        driveTrain.pushTelemetry();
    }
}
