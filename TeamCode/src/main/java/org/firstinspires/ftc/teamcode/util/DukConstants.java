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
        public static final int LIFT_INCREMENT = 10;
        public static final boolean ALLOW_WRIST_CHANGE_WHILE_TRAVEL = true;
        public static final int HANGER_INCREMENT = 100;
        public static final float CLAW_INTAKE_SPEED = 1f;
        public static float CLAW_INTAKE_DROP_TIME = 0.7f;
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
        //mm
        public static final float PIXEL_HEIGHT = 12.7f;
    }

    public static final class AUTOMATED_CONTROLLER_PARAMS {
        public static final class AUTON_BEGIN_POSES {
            //Assume froward is from the red alliance
            public static final float RED_RIGHT_X = 16000;
            public static final float RED_RIGHT_Y = -83000;
            public static final float RED_RIGHT_H = 0;
            public static final float RED_LEFT_X = -47000;
            public static final float RED_LEFT_Y = -83000;
            public static final float RED_LEFT_H = 0;
            public static final float BLUE_LEFT_X = 16000;
            public static final float BLUE_LEFT_Y = 83000;
            public static final float BLUE_LEFT_H = (float)Math.toRadians(180);
            public static final float BLUE_RIGHT_X = -47000;
            public static final float BLUE_RIGHT_Y = 83000;
            public static final float BLUE_RIGHT_H = (float)Math.toRadians(180);

        }

        public static final PIDFCalculator ROBOT_ROTATION_PID = new PIDFCalculator(1.1f, 0.0f, 100, 0.09f, 0, 0, 0, true);
        public static final PIDFCalculator ROBOT_PURSUIT_PID = new PIDFCalculator(0.0002f, 0f, 100, -0.00003f, 0, 0, 0, false);
        public static final int LIFT_MAX = 2500;
        public static final int LIFT_MIN = 1;
        public static final int LIFT_PRESET_1 = 500;
        public static final int LIFT_PRESET_2 = 1000;
        public static final int LIFT_PRESET_3 = 2500;
        public static final PIDFCalculator LIFT_PID = new PIDFCalculator(0.01f, 0f, 0f, 0f, 0f, 0, 0, false);
        //fDividend is set in PixelManagement
        public static float WRIST_SPEED = 0.08f;
        public static final float WRIST_CLAW_ACTIVATION_THRESHOLD = 0.4f;
        public static final float WRIST_CUTOFF = 0.05f;
        public static final float MAX_HANGER_POSITION = 0;
        public static final float MIN_HANGER_POSITION = -3200;
        public static final float HANGER_CUTOFF_TOLERANCE = 100;
        public static final float HANGER_LIFT_DEADZONE = 500;
        public static final float HANGER_CLEARANCE_POSITION = -300;
        public static final PIDFCalculator HANGER_PI = new PIDFCalculator(0.005f, 0, 0, 0, 0, 0, 0, false);
        public static final float STANDARD_PURSUIT_RANGE = 1300;
        public static final float STANDARD_HEADING_RANGE = 0.08f;
        public static final int MAX_PURSUIT_DELTA_POS_ET = 10;
        public static final float MAX_PURSUIT_DELTA_HEADING = 0.02f;
        public static final float LIFT_HEIGHT_WRIST_REST = 10;
        public static final float LIFT_DESCENT_WRIST_POSITION = 0.2f;
    }

    //All time units are in seconds
    public static final class ASSESSED_DIAGNOSTICS {
        public static final float MAX_ODOMETER_FAILURE_TIME = 3f;
        public static final float ODOMETER_MAX_DIRECTION_DEVIATION = (float)Math.toRadians(10);
        public static final float MIN_DRIVE_POWER = 2f;
        public static final float IMU_CHECK_INTERVAL = 1f;
        public static final float MAX_IMU_ODOMETER_DIFFERENCE = (float)Math.toRadians(36000000);
    }

    public static final class WEBCAM {
        public static final int RESOLUTION_WIDTH = 640;
        public static final int RESOLUTION_HEIGHT = RESOLUTION_WIDTH * 9 / 16;
        public static final OpenCvCameraRotation ORIENTATION =  OpenCvCameraRotation.UPRIGHT;
        public static final float PITCH_FOV = (float)Math.toRadians(43.3);
        public static final float YAW_FOV = (float)Math.toRadians(70.42);
        //[0:Lower, 1:Upper]
        public static final Map<ObservedFieldObject.Type, Scalar[]> COLOR_SCALARS = new HashMap<ObservedFieldObject.Type, Scalar[]>() {{
            put(ObservedFieldObject.Type.WHITE_PIXEL, new Scalar[]{new Scalar(0, 0, 200, Imgproc.COLOR_RGB2HSV), new Scalar(255, 30, 255)});
            put(ObservedFieldObject.Type.YELLOW_PIXEL, new Scalar[]{new Scalar(0, 130, 50, Imgproc.COLOR_RGB2YCrCb), new Scalar(255, 180, 120)});
            put(ObservedFieldObject.Type.GREEN_PIXEL, new Scalar[]{new Scalar(30, 50, 0, Imgproc.COLOR_RGB2HSV), new Scalar(70, 255, 255)});
            put(ObservedFieldObject.Type.PURPLE_PIXEL, new Scalar[]{new Scalar(0, 40, 160, Imgproc.COLOR_RGB2HSV), new Scalar(255, 100, 255)});
            put(ObservedFieldObject.Type.TEAM_PROP_RED, new Scalar[]{new Scalar(0, 0, 108, Imgproc.COLOR_RGB2YCrCb), new Scalar(255, 255, 120)});
            put(ObservedFieldObject.Type.TEAM_PROP_BLUE, new Scalar[]{new Scalar(0, 0, 150, Imgproc.COLOR_RGB2YCrCb), new Scalar(255, 100, 160)});
        }};
        public static final float HEIGHT = 88.7f;
        public static final float PITCH = (float)Math.toRadians(-40f);
        public static int IMAGE_LAYER = 0;
        public static float MIN_BOUNDING_AREA = 0.00f * RESOLUTION_WIDTH;
        public static final DecimalFormat COORD_TEXT_FORMAT = new DecimalFormat("#.###");
        public static final float SPIKE_LEFT_BOUNDARY = 0.4f * RESOLUTION_WIDTH;
        public static final float SPIKE_MIDDLE_BOUNDARY = 0.8f * RESOLUTION_WIDTH;
    }
}