package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class BetterGamepad {
    private final Gamepad gamepad;
    private boolean aPressed, bPressed, xPressed, yPressed, backPressed, leftStickPressed;

    public BetterGamepad(Gamepad _gamepad) {
        gamepad = _gamepad;
    }

    public boolean onAPressed() {
        boolean newPress = gamepad.a && !aPressed;
        aPressed = gamepad.a;
        return newPress;
    }

    public boolean onBPressed() {
        boolean newPress = gamepad.b && !bPressed;
        bPressed = gamepad.b;
        return newPress;
    }

    public boolean onXPressed() {
        boolean newPress = gamepad.x && !xPressed;
        xPressed = gamepad.x;
        return newPress;
    }

    public boolean onYPressed() {
        boolean newPress = gamepad.y && !yPressed;
        yPressed = gamepad.y;
        return newPress;
    }

    public boolean onBackPressed() {
        boolean newPress = gamepad.back && !backPressed;
        backPressed = gamepad.back;
        return newPress;
    }

    public boolean onLeftStickPress() {
        boolean newPress = gamepad.left_stick_button && !leftStickPressed;
        leftStickPressed = gamepad.left_stick_button;
        return newPress;
    }
}
