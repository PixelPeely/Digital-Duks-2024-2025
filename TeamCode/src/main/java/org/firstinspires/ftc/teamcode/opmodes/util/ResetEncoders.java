package org.firstinspires.ftc.teamcode.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.DukOpMode;

@TeleOp
public class ResetEncoders extends DukOpMode {
    @Override
    public void init() {
        super.init();
        _hardwareMap.lift.resetEncoders();
    }

    @Override
    public void preTick() {

    }

    @Override
    public void postTick() {

    }
}
