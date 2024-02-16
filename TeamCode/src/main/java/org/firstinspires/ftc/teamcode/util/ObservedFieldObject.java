package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Rect;

public class ObservedFieldObject {
    public float xRelative;
    public float yRelative;
    public float xAbsolute;
    public float yAbsolute;
    public Rect boundingRect;

    public ObservedFieldObject(float _xRelative, float _yRelative, Rect _boundingRect) {
        xRelative = _xRelative;
        yRelative = _yRelative;
        boundingRect = _boundingRect;
    }

    public enum Type {
        WHITE_PIXEL,
        YELLOW_PIXEL,
        GREEN_PIXEL,
        PURPLE_PIXEL,
        TEAM_PROP_RED,
        TEAM_PROP_BLUE
    }
}
