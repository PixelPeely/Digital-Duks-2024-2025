package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;

public class DukUtilities {

    public static float angleWrap(float a) {
        float result = a;
        while (result > Math.PI)
            result -= Math.PI;
        while (result < -Math.PI)
            result += Math.PI;
        return (float)(result == a ? a : result < 0 ? result + Math.PI : result > 0 ? result - Math.PI : 0);
    }

    public static float wrappedAngleDifference(float a, float b) {
        float distance = angleWrap(b-a);
        float inverse_distance = angleWrap(a-b);
        return -(Math.abs(distance) < Math.abs(inverse_distance) ? distance : inverse_distance);
    }

    public static double clamp(double value, double max, double min) {
        if (max < min) System.out.println("DukUtilities.clamp(" + value + ") was called with a max (" + max + ") < min(" + min + ")!");
        return Math.max(min, Math.min(max, value));
    }

    public static boolean isRotationInRange(float a, float boundaryBegin, float boundaryEnd) {
        return (boundaryBegin > 0 && boundaryEnd > 0) ? (
                    (boundaryBegin < boundaryEnd) ?
                            boundaryBegin < a && a < boundaryEnd :
                            a > boundaryBegin || a < boundaryEnd
                ) : (boundaryBegin < 0 && boundaryEnd < 0) ? (
                    (boundaryBegin > boundaryEnd) ?
                            boundaryBegin < a || a < boundaryEnd :
                            a > boundaryBegin && a < boundaryEnd
                ) : (
                    a > boundaryBegin || a < boundaryEnd
                );
    }

    public static float servoPositionToRotation(float position, float offset, float scaleLow, float scaleHigh) {
        return scaleLow + (position - offset) * 2 * (float)Math.PI * Math.abs(scaleLow - scaleHigh);
    }

    public static void mapPose(float heading, Pose toMap, Vector offset, boolean toGlobal) {
        Vector _offset = new Vector(offset);
        _offset.rotate(heading);
        if (toGlobal) _offset.negate();
        toMap.pos.add(_offset);
    }

    public static Vector ETToFieldCoords(Pose pose) {
        Vector pos = new Vector(pose.pos);
        pos.scale(0.03937f/(float)DukConstants.HARDWARE.ET_PER_MM);
        return new Vector(pos.getY(), -pos.getX());
    }
}