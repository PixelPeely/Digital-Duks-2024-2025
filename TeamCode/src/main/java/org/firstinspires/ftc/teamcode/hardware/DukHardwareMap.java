package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.assemblies.HardLink;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Clutch;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DuplexIMU;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SubmersibleIntake;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.InternalTaskInstances;
import org.firstinspires.ftc.teamcode.util.TimeManager;
import org.firstinspires.ftc.teamcode.util.autonomous.AutoTask;

import java.util.List;

public class DukHardwareMap {
    public List<LynxModule> allHubs;
    public final DriveTrain driveTrain;
    public final Clutch clutch;
    public final Lift lift;
    public final SubmersibleIntake submersibleIntake;
    public final DuplexIMU duplexIMU;

    private static DukHardwareMap instance;

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
        clutch = new Clutch(hardwareMap);
        lift = new Lift(hardwareMap);
        submersibleIntake = new SubmersibleIntake(hardwareMap);
        duplexIMU = new DuplexIMU(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        AutoTask.Base._hardwareMap = this;

        instance = this;
        InternalTaskInstances.InternalInteractions.setHardwareMapInstance(this);
    }

    public static class InternalInteractions {
        public static double getLiftPosition() {
            return instance.lift.winch.getAveragePower();
        }

        public static void extendoClearanceUpdate(boolean retracted) {
            instance.lift.extendoClearanceUpdate(retracted);
        }

        public static boolean extendoCanTransfer() {
            return instance.submersibleIntake.canTransfer();
        }

        public static void attemptTransfer() {
            if (!instance.lift.canTransfer() || !instance.submersibleIntake.canTransfer()) return;
            instance.lift.pivotDeposit.claw.setState(true);
            instance.lift.pivotDeposit.claw.dispatchCache();
            System.out.println("closed else retard");
            TimeManager.hookFuture(0.7, InternalTaskInstances.InternalInteractions.intakeRelease);
        }
    }

    public void setClutchState(Clutch.STATE state) {
        clutch.setState(state);

        if (state == Clutch.STATE.ENGAGED)
            driveTrain.forAllMotors(m -> lift.winch.addLink(new HardLink.Link(m, DukConstants.HARDWARE.DRIVETRAIN_WINCH_RATIO)));
        else
            driveTrain.forAllMotors(m -> lift.winch.removeLink(new HardLink.Link(m, DukConstants.HARDWARE.DRIVETRAIN_WINCH_RATIO)));
        //New instances might not match those of the list
    }

    public void alignWithSample(PoseEstimator.Pose samplePose) {
//        double sin = Math.sin(driveTrain.poseEstimator.getPose().getH());
//        double cos = Math.cos(driveTrain.poseEstimator.getPose().getH());
//        Vector closestPoint = new Vector(
//                samplePose.pos.getX() * cos * cos + sin * (
//
//                        )
//
//        );
    }

    public void dispatchAll() {
        driveTrain.dispatchAllCaches();
        lift.dispatchAllCaches();
        submersibleIntake.dispatchAllCaches();
        duplexIMU.dispatchAllCaches();
    }

    public void refreshAll() {
        driveTrain.refreshAllCaches();
        lift.refreshAllCaches();
        submersibleIntake.refreshAllCaches();
        duplexIMU.refreshAllCaches();
    }

    public void pushTelemetryAll() {
        driveTrain.pushTelemetry();
        lift.pushTelemetry();
        submersibleIntake.pushTelemetry();
        duplexIMU.pushTelemetry();
    }

    public void allowDispatch(boolean state) {
        driveTrain.allowDispatch(state);
        clutch.allowDispatch(state);
        lift.allowDispatch(state);
        submersibleIntake.allowDispatch(state);
    }
}
