//Adjust hardware settings in the constructors of group classes

package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.Map;

public class DukConstants {
    public static final class INPUT {
        public static final boolean MAXIMIZE_MAGNITUDE = true;
        public static final float MANUAL_DRIVE_CONTROL_MULTIPLIER = 0.5f;
        public static final float MANUAL_TURN_CONTROL_MULTIPLIER = 0.05f;
        public static final float JOYSTICK_TURN_THRESHOLD = 0.5f;
        public static float INTAKE_ROLLER_SPEED = 0.3f;
    }

    public static final class ORIENTATION {
        public static final double DIRECTION_OFFSET = Math.PI / 4.0;
        public static final boolean CORRECT_FOR_DISPLACEMENT_POS = false;
        public static final boolean CORRECT_FOR_DISPLACEMENT_ROT = false;
        public static final AxesReference AXES_REFERENCE = AxesReference.INTRINSIC;
        public static final AxesOrder AXES_ORDER = AxesOrder.XYZ;
        public static final AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;
        public static final Orientation NULL_ORIENTATION = new Orientation(
                AXES_REFERENCE, AXES_ORDER, ANGLE_UNIT,
                0, 0, 0, 0);
    }

    public static final class DEBUG {
        public static final boolean USE_FTC_DASHBOARD = true;
        public static final int PACKET_TRANSMISSION_INTERVAL = 20;
        public static final boolean OPTIMIZE_PACKETS = true;
        public static final String ROBOT_POSE_STROKE = "Red";
        public static final String ROBOT_PURSUIT_STROKE = "Blue";
        public static final String ROBOT_POINT_STROKE = "Green";
        public static final int DIRECTION_INDICATOR_LENGTH = 20;
    }

    //Measurements are in encoder ticks
    public static final class HARDWARE {
        public static final int ET_PER_WHEEL_REVOLUTION = 8192;
        public static final float WHEEL_RADIUS = 1;
        public static final double ET_PER_INCH = ET_PER_WHEEL_REVOLUTION / (WHEEL_RADIUS * 2 * Math.PI);//1,303.7972938088065906186957895476
        public static final int ET_PER_ROBOT_REVOLUTION_Y = 54178;
        public static final int ET_PER_ROBOT_REVOLUTION_X = 0; //Circumference of circle about X pivot
        public static final double OX_PIVOT_DIST_FROM_CENTER = 0;
        public static final double OX_PIVOT_CENTER_THETA = 0;
        public static final double ROBOT_SIZE_IN = 15;
    }

    public static final class AUTOMATED_CONTROLLER_PARAMS {
        public static final PIDFCalculator ROBOT_ROTATION_PID = new PIDFCalculator(1.1f, 0.0f, 100, 0.09f, 0, 0, 0, true);
        public static final PIDFCalculator ROBOT_PURSUIT_PID = new PIDFCalculator(0.0002f, 0f, 100, -0.00003f, 0, 0, 0, false);
        public static final PIDFCalculator LIFT_PID = new PIDFCalculator(0.01f, 0f, 0f, 0f, 0f, 0, 0, false);
        public static final float STANDARD_PURSUIT_RANGE = 1300;
        public static final float STANDARD_HEADING_RANGE = 0.08f;
        public static final int MAX_PURSUIT_DELTA_POS_ET = 10;
        public static final float MAX_PURSUIT_DELTA_HEADING = 0.02f;
    }

    //All time units are in seconds
    public static final class ASSESSED_DIAGNOSTICS {
        public static final float MAX_ODOMETER_FAILURE_TIME = 3f;
        public static final float ODOMETER_MAX_DIRECTION_DEVIATION = (float)Math.toRadians(10);
        public static final float MIN_DRIVE_POWER = 2f;
        public static final float IMU_CHECK_INTERVAL = 1f;
        public static final float MAX_IMU_ODOMETER_DIFFERENCE = (float)Math.toRadians(36000000);
    }
}