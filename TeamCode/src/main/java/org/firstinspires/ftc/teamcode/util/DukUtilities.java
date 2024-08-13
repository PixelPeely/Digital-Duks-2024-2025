package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;

public class DukUtilities {

    public static double angleWrap(double a) {
        double result = a;
        while (result > Math.PI)
            result -= Math.PI;
        while (result < -Math.PI)
            result += Math.PI;
        return (result == a ? a : result < 0 ? result + Math.PI : result > 0 ? result - Math.PI : 0);
    }

    public static double wrappedAngleDifference(double a, double b) {
        double distance = angleWrap(b-a);
        double inverse_distance = angleWrap(a-b);
        return -(Math.abs(distance) < Math.abs(inverse_distance) ? distance : inverse_distance);
    }

    public static double clamp(double value, double max, double min) {
        if (max < min) System.out.println("DukUtilities.clamp(" + value + ") was called with a max (" + max + ") < min(" + min + ")!");
        return Math.max(min, Math.min(max, value));
    }

    public static boolean isRotationInRange(double a, double boundaryBegin, double boundaryEnd) {
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

    public static double servoPositionToRotation(double position, double offset, double scaleLow, double scaleHigh) {
        return scaleLow + (position - offset) * 2 * Math.PI * Math.abs(scaleLow - scaleHigh);
    }

    public static void mapPose(double heading, Pose toMap, Vector offset, boolean toGlobal) {
        Vector _offset = new Vector(offset);
        _offset.rotate(heading);
        if (toGlobal) _offset.negate();
        toMap.pos.add(_offset);
    }

    public static Vector ETToFieldCoords(Pose pose) {
        Vector pos = new Vector(pose.pos);
        pos.scale(0.03937f/DukConstants.HARDWARE.ET_PER_MM);
        return new Vector(pos.getY(), -pos.getX());
    }
}