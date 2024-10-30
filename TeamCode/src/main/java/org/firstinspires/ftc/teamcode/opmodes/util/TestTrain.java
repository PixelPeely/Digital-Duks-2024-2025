package org.firstinspires.ftc.teamcode.opmodes.util;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.DukOpMode;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp
public class TestTrain extends DukOpMode {
    Servo claw;
    Servo wrist;
    Servo swerve;

    @Override
    public void init() {
        super.init();
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        swerve = hardwareMap.get(Servo.class, "swerve");
    }

    float wristP = 0.5f;
    float swerveP = 0.5f;
    @Override
    public void preTick() {
        if (gamepad1.a) {
            claw.setPosition(0.5);
        } else claw.setPosition(0.1);

        wristP += gamepad1.left_stick_y * 0.01;
        swerveP +=gamepad1.left_stick_x * 0.01;
        wrist.setPosition(wristP);
        swerve.setPosition(swerveP);
    }

    @Override
    public void postTick() {

    }

    @Override
    public void stop() {
        super.stop();
    }
}
