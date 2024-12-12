package org.firstinspires.ftc.teamcode.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.DukOpMode;

@TeleOp
public class SensorSim extends DukOpMode {
    private final int numSensors = 3;
    private int sensorIndex = 0;
    private int multiple = 1;
    private double[] values = new double[numSensors];

    @Override
    public void init() {
        super.init();
        _hardwareMap.driveTrain.poseEstimator.deadWheelLocalizer.yLeft.simRoutine = motor -> motor.C_setCurrentPosition((int)values[0]);
        _hardwareMap.driveTrain.poseEstimator.deadWheelLocalizer.x.simRoutine = motor -> motor.C_setCurrentPosition((int)values[1]);
        _hardwareMap.driveTrain.poseEstimator.deadWheelLocalizer.yRight.simRoutine = motor -> motor.C_setCurrentPosition((int)values[2]);
    }

    @Override
    public void preTick() {
        sensorIndex += (gamepad1Ext.onDPadRightPressed() ? 1 : 0) - (gamepad1Ext.onDPadLeftPressed() ? 1 : 0);
        if (sensorIndex > numSensors - 1) sensorIndex = 0;
        else if (sensorIndex < 0) sensorIndex = numSensors - 2;
        double sum = gamepad1.right_trigger - gamepad1.left_trigger;
        values[sensorIndex] += sum * multiple;

        if (gamepad1Ext.onDPadUpPressed()) multiple *= 2;
        else if (gamepad1Ext.onDPadDownPressed()) multiple /= 2;

        telemetry.addData("Sensor", sensorIndex);
        telemetry.addData("Value", values[sensorIndex]);
        telemetry.addData("Multiple", multiple);
    }

    @Override
    public void postTick() {

    }
}
