package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;
import org.firstinspires.ftc.teamcode.opmodes.DukOpMode;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.util.Logger;

@Config
@TeleOp
public class TestTrain extends OpMode {
    C_Servo extendoR, extendoL;
    public static double pos;

    @Override
    public void init() {
        extendoR = new C_Servo(hardwareMap.get(Servo.class, "pivotR"));
        extendoL = new C_Servo(hardwareMap.get(Servo.class, "pivotL"));

        extendoL.setScaleRange(1, 0.2);
        extendoR.setScaleRange(0, 0.8);
    }

    @Override
    public void loop() {
        extendoL.setPosition(pos);
        extendoR.setPosition(pos);

        telemetry.addData("posL", extendoL.getPosition());
        telemetry.addData("posR", extendoR.getPosition());

        extendoL.dispatchCache();
        extendoR.dispatchCache();
    }
}
