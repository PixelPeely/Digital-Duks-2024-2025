package org.firstinspires.ftc.teamcode.util;

public class Vector {
    private double x;
    private double y;
    private double r;
    private double t;

    public Vector(double _x, double _y) {
        setX(_x);
        setY(_y);
    }

    public Vector(double _r, double _t, boolean polar) {
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
    public double getX() {
        if (x == 0 && r != 0) x = r * Math.sin(t);
        return x;
    }

    public void setX(double _x) {
        x = _x;
        r = t = 0;
    }

    public double getY() {
        if (y == 0 && r != 0) y = r * Math.cos(t);
        return y;
    }

    public void setY(double _y) {
        y = _y;
        r = t = 0;
    }

    public double getR() {
        if (r == 0 && (x != 0 || y != 0)) r = Math.sqrt(x * x + y * y);
        return r;
    }

    public double getT() {
        if (t == 0 && (x != 0 || y != 0)) t = Math.atan2(x, y);
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

    public void scale(double a) {
        setX(getX() * a);
        setY(getY() * a);
    }

    public double dot(Vector a) {
        return getX() * a.getX() + getY() * a.getY();
    }

    public double cross(Vector a) {
        return getX() * a.getY() - getY() * a.getX();
    }
    //endregion

    public void rotate(double angle) {
        t = DukUtilities.angleWrap(getT() + angle);
        r = getR();
        setX(0);
        setY(0);
    }

    public void negate() {
        setX(-getX());
        setY(-getY());
    }

    public double distance(Vector _a) {
        Vector a = new Vector(_a);
        a.subtract(this);
        return a.getR();
    }
}
