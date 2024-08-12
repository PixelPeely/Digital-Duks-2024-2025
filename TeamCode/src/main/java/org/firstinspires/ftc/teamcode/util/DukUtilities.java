package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.subsystems.PoseEstimator.Pose;

public class DukUtilities {
    public static class Vector {
        private float x;
        private float y;
        private float r;
        private float t;

        public Vector(float _xr, float _yt, boolean cartesian) {
            if (cartesian) {
                x = _xr;
                y = _yt;
            } else {
                r = _xr;
                t = _yt;
            }
        }

        public Vector(Vector vector) {
            x = vector.x;
            y = vector.y;
            r = vector.r;
            t = vector.t;
        }

        public float getX() {
            if (x == 0 && r != 0) x = r * (float)Math.sin(t);
            return x;
        }

        public float getY() {
            if (y == 0 && r != 0) y = r * (float)Math.cos(t);
            return y;
        }

        public float getR() {
            if (r == 0 && (x != 0 || y != 0)) r = (float)Math.sqrt(x * x + y * y);
            return r;
        }

        public float getT() {
            if (t == 0 && (x != 0 || y != 0)) t = (float)Math.atan2(x, y);
            return t;
        }

        public void rotate(float angle) {
            t = constrainAxis(getT() + angle);
            r = getR();
            x = y = 0;
        }
    }

    public static float constrainAxis(float a) {
        float result = a;
        while (result > Math.PI)
            result -= Math.PI;
        while (result < -Math.PI)
            result += Math.PI;
        return (float)(result == a ? a : result < 0 ? result + Math.PI : result > 0 ? result - Math.PI : 0);
    }

    public static float differenceConstrained(float a, float b) {
        float distance = constrainAxis(b-a);
        float inverse_distance = constrainAxis(a-b);
        return -(Math.abs(distance) < Math.abs(inverse_distance) ? distance : inverse_distance);
    }

    public static double clamp(double value, double max, double min) {
        if (max < min) System.out.println("DukUtilities.clamp(" + value + ") was called with a max (" + max + ") < min(" + min + ")!");
        return Math.max(min, Math.min(max, value));
    }

    public static float getJoystickDirection(Gamepad gamepad, boolean left) {
        if (left)
            return (float)Math.atan2(gamepad.left_stick_x, -gamepad.left_stick_y);
        else
            return (float)Math.atan2(gamepad.right_stick_x, -gamepad.right_stick_y);
    }

    public static float getJoystickMagnitude(Gamepad gamepad, boolean left) {
        float sum = Math.abs(left ? gamepad.left_stick_x : gamepad.right_stick_x)
                + Math.abs(left ? gamepad.left_stick_y : gamepad.right_stick_y);
        return sum / Math.max(sum, 1);
    }

    public static float getDistance(float x, float y, float dx, float dy) {
        float diffX = dx-x;
        float diffY = dy-y;
        return (float)Math.sqrt(diffX*diffX + diffY*diffY);
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
        toMap.x += _offset.getX() * (toGlobal ? -1 : 1);
        toMap.y += _offset.getY() * (toGlobal ? -1 : 1);
    }

    public static Vector ETToFieldCoords(Pose pose) {
        return new Vector(pose.y * (0.03937f/(float)DukConstants.HARDWARE.ET_PER_MM), -pose.x * (0.03937f/(float)DukConstants.HARDWARE.ET_PER_MM), true);
    }
}