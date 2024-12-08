package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class PowerTest extends OpMode {
    Servo s0, s1, s2, s3, s4, s5, s6, s7, s8, s9;
    DcMotor liftL;
    DcMotor liftR;
    public static double pos01, pos23, pos4, pos5, pos6, pos7, pos8, pos9;
    public static double liftPower;

    @Override
    public void init() {
        s0 = hardwareMap.get(Servo.class, "s0");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        s5 = hardwareMap.get(Servo.class, "s5");
        s6 = hardwareMap.get(Servo.class, "s6");
        s7 = hardwareMap.get(Servo.class, "s7");
        s8 = hardwareMap.get(Servo.class, "s8");
        s9 = hardwareMap.get(Servo.class, "s9");
        liftL = hardwareMap.get(DcMotor.class, "m0");
        liftR = hardwareMap.get(DcMotor.class, "m1");
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        s0.setPosition(1 - pos01);
        s1.setPosition(pos01);
        s2.setPosition(1 - pos23);
        s3.setPosition(pos23);
        s4.setPosition(pos4);
        s5.setPosition(pos5);
        s6.setPosition(pos6);
        s7.setPosition(pos7);
        s8.setPosition(pos8);
        s9.setPosition(pos9);
        liftL.setPower(liftPower);
        liftR.setPower(liftPower);
    }
}
