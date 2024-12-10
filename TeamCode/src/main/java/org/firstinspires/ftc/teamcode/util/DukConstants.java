//Adjust hardware settings in the constructors of group classes

package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DukConstants {
    public static final class INPUT {
        public static final double MANUAL_DRIVE_CONTROL_MULTIPLIER = 0.5f;
        public static final double MANUAL_TURN_CONTROL_MULTIPLIER = 0.05f;
        public static final double JOYSTICK_TURN_THRESHOLD = 0.5f;
    }

    public static final class ORIENTATION {
        public static final AxesReference AXES_REFERENCE = AxesReference.INTRINSIC;
        public static final AxesOrder AXES_ORDER = AxesOrder.XYZ;
        public static final AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;
        public static final Orientation NULL_ORIENTATION = new Orientation(
                AXES_REFERENCE, AXES_ORDER, ANGLE_UNIT,
                0, 0, 0, 0);
        public static final double IMU_CHECK_INTERVAL = 1f;
    }

    public static final class DEBUG {
        public static final class STROKES {
            public static final String ROBOT_POSE_STROKE = "Brown";
            public static final String ROBOT_PURSUIT_STROKE = "Blue";
            public static final String ODOMETER_WHEELS_STROKE = "Red";
        }
        public static final boolean USE_FTC_DASHBOARD = true;
        public static final int PACKET_TRANSMISSION_INTERVAL = 20;
        public static final boolean OPTIMIZE_PACKETS = true;
        public static final int DIRECTION_INDICATOR_LENGTH = 20;
        public static final String LOG_DIRECTORY = "/sdcard/FIRST/java/src/runtimeLogs";
    }

    //Measurements are in encoder ticks
    public static final class HARDWARE {
        public static final double ET_PER_MM = 4096 / (35 * Math.PI);
        public static final int ET_PER_ROBOT_REVOLUTION = 19500;//Circumference of pivot circle
        //Intersection of parallel center axis with perpendicular axis
        public static final Vector ODOMETER_CENTER = new Vector(0, -45 * ET_PER_MM);
        public static final double ROBOT_SIZE_IN = 14.5;
        public static final Vector LEFT_WHEEL_PAIR_PROFILE = new Vector(Math.sqrt(2),Math.sqrt(2));
        public static final Vector RIGHT_WHEEL_PAIR_PROFILE = new Vector(-Math.sqrt(2),Math.sqrt(2));
        public static final double EXTENDO_RET_ANGLE = Math.toRadians(85);
        public static final double EXTENDO_EXT_ANGLE = Math.toRadians(5);
        public static final double EXTENDO_LINKAGE_LENGTH = -1;
        public static final double CARRIAGE_RET_ANGLE = Math.toRadians(85);
        public static final double CARRIAGE_EXT_ANGLE = Math.toRadians(5);
        public static final double CARRIAGE_LINKAGE_LENGTH = -1;
        public static final double SUBMERSIBLE_DEAD_LENGTH = -1; //Length when fully retracted
        public static final int MIN_LIFT_HEIGHT = 250;
        public static final int MAX_LIFT_HEIGHT = 4000;
        public static final double CLIP_DISTANCE_TRANSFER = 1000;
        public static final double DRIVETRAIN_WINCH_RATIO = 29.0/30.0;
    }

    public static final class AUTOMATED_CONTROLLER_PARAMS {
        public static final PIDFCalculator ROBOT_ROTATION_PID = new PIDFCalculator(1.1, 0.0, 100, 0.09, 0, 0, 0, true);
        public static final PIDFCalculator ROBOT_PURSUIT_PID = new PIDFCalculator(0.0002, 0, 100, -0.00003, 0, 0, 0, false);
        public static final PIDFCalculator LIFT_PIDF = new PIDFCalculator(0.005, 0, 0, 0);
        public static final double STANDARD_PURSUIT_RANGE = 1300;
        public static final double STANDARD_HEADING_RANGE = 0.08f;
        public static final int MAX_PURSUIT_SPEED = 10;
        public static final double MAX_PURSUIT_ANGULAR_SPEED = 0.02f;
        public static final int LIFT_ERROR = 100;
    }
}