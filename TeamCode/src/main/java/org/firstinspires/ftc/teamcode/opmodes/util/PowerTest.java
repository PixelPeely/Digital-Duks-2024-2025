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
    public static double pos;
    public static Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "shuttleClaw");
    }

    @Override
    public void loop() {
        servo.setPosition(pos);
    }
}
