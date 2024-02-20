package org.firstinspires.ftc.teamcode.util;

//This is a generalized version of the Digital Duks Odometry System
public class WheelOdometry {
    //How many ticks are measured by the odometers for a full revolution of the robot
    //(distance between parallel odometers) * (encoder ticks per revolution; 8192) / (diameter of odometer wheel)
    private static final double TICKS_PER_ROBOT_REVOLUTION = 60000;

    //The number of ticks measured by the parallel odometers per revolution, if their inputs were averaged
    //(horizontal distance to center of parallel odometers) * (encoder ticks per revolution; 8192) / (diameter of odometer wheel)
    private static final double TICKS_PER_PARALLEL_ODOMETER_REVOLUTION = 0;

    //Number of ticks the perpendicular odometer measures during a full revolution
    //(vertical distance to perpendicular wheel) * (encoder ticks per revolution; 8192) / (diameter of odometer wheel)
    private static final double TICKS_PER_PERP_REVOLUTION = 0;


    //Positions of the robot (in encoder ticks)
    public static double poseX;
    public static double poseY;
    //Delta positions of the robot (how far it traveled since last compute call)
    public static double poseDX;
    public static double poseDY;

    public static double heading;
    public static double headingDelta;


    private static double leftLastET;
    private static double rightLastET;
    private static double perpLastET;

    private static double constrainAxis(double a) {
        double result = a;
        while (result > Math.PI)
            result -= Math.PI;
        while (result < -Math.PI)
            result += Math.PI;
        return (result == a ? a : result < 0 ? result + Math.PI : result > 0 ? result - Math.PI : 0);
    }


    /**
     * Clear all current values stored in the class; this is helpful for resetting the robot between opmodes
     */
    public static void reset() {
        poseX = poseY = poseDX = poseDY = heading = headingDelta = leftLastET = rightLastET = perpLastET = 0;
    }

    /**
     * Calculate the amount moved since the last call to this function, and update poses accordingly
     * @param leftOdometer Current encoder reading of the left odometer
     * @param rightOdometer Current encoder reading of the right odometer
     * @param perpOdometer Current encoder reading of the perpendicular odometer
     */
    public static void computePoseDelta(double leftOdometer, double rightOdometer, double perpOdometer) {
        double dLeftOdometer = leftOdometer - leftLastET;
        double dRightOdometer = rightOdometer - rightLastET;
        double dPerpOdometer = perpOdometer - perpLastET;

        //Heading
        double arcPercent = (dLeftOdometer - dRightOdometer) / (2 * TICKS_PER_ROBOT_REVOLUTION);
        headingDelta = 2 * Math.PI * arcPercent;
        heading = constrainAxis(heading + headingDelta);
        double averageHeading = heading - headingDelta * 0.5;

        //Position
        double yTicks = (dLeftOdometer + dRightOdometer) * 0.5 - arcPercent * TICKS_PER_PARALLEL_ODOMETER_REVOLUTION;
        double xTicks = dPerpOdometer - arcPercent * TICKS_PER_PERP_REVOLUTION;
        double hCos = Math.cos(averageHeading);
        double hSin = Math.sin(averageHeading);
        poseDX = hCos * yTicks - hSin * xTicks;
        poseDY = hSin * yTicks + hCos * xTicks;
        poseX += poseDX;
        poseY += poseDY;

        leftLastET = leftOdometer;
        rightLastET = rightOdometer;
        perpLastET = perpOdometer;
    }

    /**
     * @param x Point x
     * @param y Point y
     * @return Direction from robot to point in radians
     */
    public static double getDirectionToPoint(double x, double y) {
        return Math.atan2(x - poseX, y - poseY);
    }

    /**
     * @param x Point X
     * @param y Point Y
     * @return Distance from robot to point in encoder ticks
     */
    public static double getDistanceToPoint(double x, double y) {
        return Math.sqrt(Math.pow(x - poseX, 2) + Math.pow(y - poseY, 2));
    }

    /**
     * @return The magnitude of the robot's movement since last pose computation
     */
    public static double getDeltaMagnitude() {
        return Math.sqrt(poseDX*poseDX + poseDY*poseDY);
    }
}
