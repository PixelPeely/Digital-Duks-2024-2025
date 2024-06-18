package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadExt extends Gamepad {
    private boolean aPressed, bPressed, xPressed, yPressed, backPressed, leftStickPressed;
    Gamepad base;

    public GamepadExt(Gamepad base) {
        this.base = base;
    }

    public boolean onAPressed() {
        boolean newPress = base.a && !aPressed;
        aPressed = base.a;
        return newPress;
    }

    public boolean onBPressed() {
        boolean newPress = b && !bPressed;
        bPressed = b;
        return newPress;
    }

    public boolean onXPressed() {
        boolean newPress = x && !xPressed;
        xPressed = x;
        return newPress;
    }

    public boolean onYPressed() {
        boolean newPress = y && !yPressed;
        yPressed = y;
        return newPress;
    }

    public boolean onBackPressed() {
        boolean newPress = back && !backPressed;
        backPressed = back;
        return newPress;
    }

    public boolean onLeftStickPress() {
        boolean newPress = left_stick_button && !leftStickPressed;
        leftStickPressed = left_stick_button;
        return newPress;
    }
}
