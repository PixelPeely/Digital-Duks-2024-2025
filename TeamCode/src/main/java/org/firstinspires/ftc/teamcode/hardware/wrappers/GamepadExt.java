package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.TimeManager;
import org.firstinspires.ftc.teamcode.util.Vector;

public class GamepadExt extends Gamepad {
    private boolean aPressed, bPressed, xPressed, yPressed, backPressed, leftStickPressed;
    Gamepad base;
    public Vector leftJoystick, rightJoystick;

    public GamepadExt(Gamepad _base) {
        base = _base;
        TimeManager.hookTick(t -> {
            leftJoystick = new Vector(base.left_stick_x, -base.left_stick_y);
            rightJoystick = new Vector(base.right_stick_x, -base.right_stick_y);
            return false;
        });
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
