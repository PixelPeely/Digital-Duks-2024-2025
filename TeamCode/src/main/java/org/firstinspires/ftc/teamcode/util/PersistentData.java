package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;

public class PersistentData {
    public static boolean available = false;
    public static Pose pose;


    public static void apply(DukHardwareMap hMap) {
        if (available) {
            hMap.driveTrain.poseEstimator.setPose(pose);
        }
        //Todo estimate the pose if velocity is not zero
    }
}
