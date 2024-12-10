package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.assemblies.HardLink;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shuttle;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SubmersibleIntake;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_TelemetryLoggingBuffer;
import org.firstinspires.ftc.teamcode.opmodes.DukOpMode;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.util.Logger;

@Config
@TeleOp
public class TestTrain extends DukOpMode {
//    C_Servo linkageL;
//    public static double pos;
//    @Override
//    public void init() {
//        linkageL = new C_Servo(hardwareMap.get(Servo.class, "shuttlePitch"));
//        linkageL.setScaleRange(1, 0);
//    }
//
//    @Override
//    public void loop() {
//        linkageL.setPosition(pos);
//        linkageL.dispatchCache();
//    }
    public static Lift.STATE liftState = Lift.STATE.DOWN;
    public static SubmersibleIntake.STATE intakeState = SubmersibleIntake.STATE.DROP;
    public static Shuttle.STATE shuttleState = Shuttle.STATE.DROP;

    private static Lift.STATE prevLiftState = liftState;
    private static SubmersibleIntake.STATE prevIntakeState = intakeState;
    private static Shuttle.STATE prevShuttleState = shuttleState;

    @Override
    public void preTick() {
        if (prevLiftState != liftState)
            _hardwareMap.lift.setState(liftState);
        if (prevIntakeState != intakeState)
            _hardwareMap.submersibleIntake.setState(intakeState);
        if (prevShuttleState != shuttleState)
            _hardwareMap.submersibleIntake.shuttle.setState(shuttleState);

        prevLiftState = liftState;
        prevIntakeState = intakeState;
        prevShuttleState = shuttleState;
    }

    @Override
    public void postTick() {

    }
}
