package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SwerveBasicTest extends OpMode {
    DcMotor motor;
    AnalogInput input;
    CRServo servo;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "frontLeft");
        input = hardwareMap.get(AnalogInput.class, "analogInput");
        servo = hardwareMap.get(CRServo.class, "servo");
    }

    @Override
    public void loop() {
        motor.setPower((gamepad1.right_trigger - gamepad1.left_trigger) * 0.2);
        servo.setPower((gamepad1.right_bumper ? 1 : -(gamepad1.left_bumper ? 1 : 0)));
        telemetry.addData("Voltage", input.getVoltage());
        telemetry.update();
    }
}
