package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedSubsystem;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_RevDistanceSensor;
import org.firstinspires.ftc.teamcode.util.DukConstants;

public class ClipSensor implements CachedSubsystem {
    public final C_RevDistanceSensor left, down, right;

    enum POSITION {
        DOWN,
        LEFT,
        RIGHT,
        UNKNOWN
    }

    public ClipSensor(HardwareMap hardwareMap) {
        down = new C_RevDistanceSensor(hardwareMap.tryGet(Rev2mDistanceSensor.class, "clipDownSensor"));
        left = new C_RevDistanceSensor(hardwareMap.tryGet(Rev2mDistanceSensor.class, "clipLeftSensor"));
        right = new C_RevDistanceSensor(hardwareMap.tryGet(Rev2mDistanceSensor.class, "clipRightSensor"));
    }

    public POSITION detectClipPosition() {
        POSITION position = POSITION.LEFT;
        C_RevDistanceSensor shortestDistance = left;

        if (down.getDistance() < shortestDistance.getDistance()) {
            position = POSITION.DOWN;
            shortestDistance = down;
        }
        else if (right.getDistance() < shortestDistance.getDistance()) {
            position = POSITION.RIGHT;
            shortestDistance = right;
        }

        return shortestDistance.getDistance() < DukConstants.HARDWARE.CLIP_DISTANCE_TRANSFER ? position : POSITION.UNKNOWN;
    }

    @Override
    public void dispatchAllCaches() {

    }

    @Override
    public void refreshAllCaches() {
        down.refreshCache();
        left.refreshCache();
        right.refreshCache();
    }

    @Override
    public void pushTelemetry() {

    }

    @Override
    public void allowDispatch(boolean state) {
        down.allowDispatch(state);
        left.allowDispatch(state);
        right.allowDispatch(state);
    }
}
