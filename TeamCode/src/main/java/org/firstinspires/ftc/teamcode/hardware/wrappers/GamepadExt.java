package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.TimeManager;
import org.firstinspires.ftc.teamcode.util.Vector;

public class GamepadExt {
    private boolean aPressed, bPressed, xPressed, yPressed,
            rightBumperPressed, leftBumperPressed,
            dPadUpPressed, dPadRightPressed, dPadDownPressed, dPadLeftPressed,
            backPressed,
            leftStickPressed;
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
        boolean newPress = base.b && !bPressed;
        bPressed = base.b;
        return newPress;
    }

    public boolean onXPressed() {
        boolean newPress = base.x && !xPressed;
        xPressed = base.x;
        return newPress;
    }

    public boolean onYPressed() {
        boolean newPress = base.y && !yPressed;
        yPressed = base.y;
        return newPress;
    }

    public boolean onBackPressed() {
        boolean newPress = base.back && !backPressed;
        backPressed = base.back;
        return newPress;
    }

    public boolean onLeftStickPress() {
        boolean newPress = base.left_stick_button && !leftStickPressed;
        leftStickPressed = base.left_stick_button;
        return newPress;
    }

    public boolean onLeftBumperPressed() {
        boolean newPress = base.left_bumper && !leftBumperPressed;
        leftBumperPressed = base.left_bumper;
        return newPress;
    }

    public boolean onRightBumperPressed() {
        boolean newPress = base.right_bumper && !rightBumperPressed;
        rightBumperPressed = base.right_bumper;
        return newPress;
    }

    public boolean onDPadUpPressed() {
        boolean newPress = base.dpad_up && !dPadUpPressed;
        dPadUpPressed = base.dpad_up;
        return newPress;
    }

    public boolean onDPadRightPressed() {
        boolean newPress = base.dpad_right && !dPadRightPressed;
        dPadRightPressed = base.dpad_right;
        return newPress;
    }

    public boolean onDPadDownPressed() {
        boolean newPress = base.dpad_down && !dPadDownPressed;
        dPadDownPressed = base.dpad_down;
        return newPress;
    }

    public boolean onDPadLeftPressed() {
        boolean newPress = base.dpad_left && !dPadLeftPressed;
        dPadLeftPressed = base.dpad_left;
        return newPress;
    }

    public float getTriggerDifference() {
        return base.right_trigger - base.left_trigger;
    }
}
