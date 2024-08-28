package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_AAEServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_AnalogInput;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_CRServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.util.PIDFCalculator;

//TODO Delete this class! It is not in accordance with the standard programming procedure for Caches!
@Config
public class SwerveTest extends DukOpMode{
    SwerveModule swerveModule;
    public static double azimuthP;
    public static double azimuthI;
    public static double azimuthD;

    @Override
    public void init() {
        super.init();
        swerveModule = new SwerveModule(
                new C_DcMotor(hardwareMap.get(DcMotorEx.class, "motor")),
                new C_AAEServo(
                        new C_CRServo(hardwareMap.get(CRServo.class, "servo")),
                        new C_AnalogInput(hardwareMap.get(AnalogInput.class, "absoluteEncoder")),
                        new PIDFCalculator(0, 0, 10, 0, true)
                ),
                0
        );
    }

    @Override
    public void preTick() {
        swerveModule.servo.pidf.P = azimuthP;
        swerveModule.servo.pidf.I = azimuthI;
        swerveModule.servo.pidf.D = azimuthD;
        swerveModule.setTargetState(gamepad1Ext.leftJoystick);
    }

    @Override
    public void postTick() {
        swerveModule.dispatchAllCaches();
    }
}
