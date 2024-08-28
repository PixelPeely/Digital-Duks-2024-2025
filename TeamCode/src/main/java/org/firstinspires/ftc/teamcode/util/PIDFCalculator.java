package org.firstinspires.ftc.teamcode.util;

public class PIDFCalculator {
    public double P;
    public double I;
    public double I_MAX;
    public double D;
    public double F;
    public double FOffset;
    public double FDividend;
    public boolean wrapDifference;

    private double lastState = 0;
    public double target = 0;
    private double error = 0;

    public PIDFCalculator(double p, double i, double i_max, double d, double f, double fOffset, double fDividend, boolean _wrapDifference) {
        P = p;
        I = i;
        I_MAX = i_max;
        D = d;
        F = f;
        FOffset = fOffset; //Orientation when erect relative to init position
        FDividend = f == 0 ? 1 : fDividend; //Usually ticks per revolution
        wrapDifference = _wrapDifference;
    }

    //region Alternate constructors
    public PIDFCalculator(double p, double i, double i_max, double d) {
        new PIDFCalculator(p, i, i_max, d, 0, 0, 0, false);
    }

    public PIDFCalculator(double p, double i, double i_max, double d, boolean _wrapDifference) {
        new PIDFCalculator(p, i, i_max, d, 0, 0, 0, _wrapDifference);
    }
    //endregion

    public double evaluate(double currentState) {
        double difference = wrapDifference ? DukUtilities.wrappedAngleDifference(target, currentState) : target - currentState;
        double delta = wrapDifference ? DukUtilities.wrappedAngleDifference(lastState, currentState) : currentState - lastState;
        double p = P * difference;
        double i = I * error;
        double d = D * delta / TimeManager.getDeltaTime();
        double f = F * -Math.sin((currentState - FOffset) * (2 * Math.PI / FDividend));

        lastState = currentState;
        error = DukUtilities.clamp(error + difference, I_MAX, -I_MAX);

        return p + i + d + f;
    }
}
