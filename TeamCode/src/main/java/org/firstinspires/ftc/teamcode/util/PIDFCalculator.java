package org.firstinspires.ftc.teamcode.util;

public class PIDFCalculator {
    public float P;
    public float I;
    public float I_MAX;
    public float D;
    public float F;
    public float FOffset;
    public float FDividend;
    public boolean constrainDifference;

    private float lastState = 0;
    public float target = 0;
    private float error = 0;

    public PIDFCalculator(float p, float i, float i_max, float d, float f, float fOffset, float fDividend, boolean _constrainDifference) {
        P = p;
        I = i;
        I_MAX = i_max;
        D = d;
        F = f;
        FOffset = fOffset; //Orientation when erect relative to init position
        FDividend = f == 0 ? 1 : fDividend; //Usually ticks per revolution
        constrainDifference = _constrainDifference;
    }

    public float evaluate(float currentState) {
        float difference = constrainDifference ? DukUtilities.differenceConstrained(target, currentState) : target - currentState;
        float delta = constrainDifference ? DukUtilities.differenceConstrained(lastState, currentState) : currentState - lastState;
        float p = P * difference;
        float i = I * error;
        float d = D * delta / (float)TimeManager.getDeltaTime();
        float f = F * -(float)Math.sin((currentState - FOffset) * (2 * Math.PI / FDividend));

        lastState = currentState;
        error = (float)DukUtilities.clamp(error + difference, I_MAX, -I_MAX);

        return p + i + d + f;
    }
}
