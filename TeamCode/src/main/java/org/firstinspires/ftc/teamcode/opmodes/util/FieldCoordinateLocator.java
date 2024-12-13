package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator;
import org.firstinspires.ftc.teamcode.opmodes.DukOpMode;
import org.firstinspires.ftc.teamcode.util.Vector;

@TeleOp
@Config
public class FieldCoordinateLocator extends DukOpMode {
    public static double posX, posY;

    @Override
    public void init() {
        super.init();
        _hardwareMap.driveTrain.allowDispatch(false);
    }

    @Override
    public void preTick() {
        _hardwareMap.driveTrain.poseEstimator.setPose(new PoseEstimator.Pose(new Vector(posX, posY)));
    }

    @Override
    public void postTick() {

    }
}
