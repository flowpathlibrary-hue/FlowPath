package com.flowpath;

/** PID with filtered derivative and clamped integral. Gains are pushed in by Flow every loop. */
public final class PID {
    public double kP, kI, kD;
    private double integral, lastErr, deriv;
    private boolean first = true;

    public void set(double p, double i, double d) { kP = p; kI = i; kD = d; }

    public void reset() { integral = 0; lastErr = 0; deriv = 0; first = true; }

    public double calculate(double error, double dt) {
        if (first) { lastErr = error; first = false; }
        if (kI != 0) {
            integral += error * dt;
            double lim = Tune.Filter.integralMax / Math.abs(kI);
            integral = MathUtil.clamp(integral, -lim, lim);
        } else integral = 0;
        double raw = (error - lastErr) / dt;
        deriv += Tune.Filter.derivativeAlpha * (raw - deriv);
        lastErr = error;
        return kP * error + kI * integral + kD * deriv;
    }
}
