package org.firstinspires.ftc.teamcode.util;

public class Vector {
    private float x;
    private float y;
    private float r;
    private float t;

    public Vector(float _x, float _y) {
        setX(_x);
        setY(_y);
    }

    public Vector(float _r, float _t, boolean polar) {
        r = _r;
        t = _t;
    }

    public Vector(Vector vector) {
        setX(vector.x);
        setY(vector.y);
        r = vector.r;
        t = vector.t;
    }

    //region Setters and Getteres
    public float getX() {
        if (x == 0 && r != 0) x = r * (float)Math.sin(t);
        return x;
    }

    public void setX(float _x) {
        x = _x;
        r = t = 0;
    }

    public float getY() {
        if (y == 0 && r != 0) y = r * (float)Math.cos(t);
        return y;
    }

    public void setY(float _y) {
        y = _y;
        r = t = 0;
    }

    public float getR() {
        if (r == 0 && (x != 0 || y != 0)) r = (float)Math.sqrt(x * x + y * y);
        return r;
    }

    public float getT() {
        if (t == 0 && (x != 0 || y != 0)) t = (float)Math.atan2(x, y);
        return t;
    }
    //endregion

    //region Arithmetic
    public void add(Vector a) {
        setX(getX() + a.getX());
        setY(getY() + a.getY());
    }

    public void subtract(Vector a) {
        setX(getX() - a.getX());
        setY(getY() - a.getY());
    }

    public void scale(float a) {
        setX(getX() * a);
        setY(getY() * a);
    }

    public float dot(Vector a) {
        return getX() * a.getX() + getY() * a.getY();
    }

    public float cross(Vector a) {
        return getX() * a.getY() - getY() * a.getX();
    }
    //endregion

    public void rotate(float angle) {
        t = DukUtilities.angleWrap(getT() + angle);
        r = getR();
        setX(0);
        setY(0);
    }

    public void negate() {
        setX(-getX());
        setY(-getY());
    }

    public float distance(Vector _a) {
        Vector a = new Vector(_a);
        a.subtract(this);
        return a.getR();
    }
}
