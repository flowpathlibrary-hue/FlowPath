package com.flowpath;

import java.util.ArrayList;
import java.util.List;

/** One curve of a Route plus its heading rule, power cap and callbacks. Built by RouteBuilder. */
public final class Segment {
    public enum Heading { LINEAR, TANGENT, REVERSED, HOLD }

    public static final class Callback {
        public final double t;
        public final Runnable action;
        Callback(double t, Runnable action) { this.t = t; this.action = action; }
    }

    private static final int SAMPLES = 200;

    public final Curve curve;
    Heading headingRule = Heading.LINEAR;
    double startHeading, endHeading;
    double maxPower = -1; // < 0 => Tune.General.maxPower
    final List<Callback> callbacks = new ArrayList<>();
    private final double[] arc = new double[SAMPLES + 1];

    Segment(Curve curve, double startHeading, double endHeading) {
        this.curve = curve;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
        Vec prev = curve.point(0);
        for (int i = 1; i <= SAMPLES; i++) {
            Vec q = curve.point(i / (double) SAMPLES);
            arc[i] = arc[i - 1] + q.minus(prev).norm();
            prev = q;
        }
    }

    public double length() { return arc[SAMPLES]; }

    /** Arc length from the start up to parameter t. */
    public double lengthAt(double t) {
        double x = MathUtil.clamp(t, 0, 1) * SAMPLES;
        int i = (int) Math.min(Math.floor(x), SAMPLES - 1);
        return arc[i] + (arc[i + 1] - arc[i]) * (x - i);
    }

    public double maxPower() { return maxPower > 0 ? maxPower : Tune.General.maxPower; }
    public List<Callback> callbacks() { return callbacks; }

    /** Closest t on the curve, searched forward only within [fromT, fromT + window]. */
    public double closestT(Vec pos, double fromT, double window) {
        double lo = MathUtil.clamp(fromT, 0, 1), hi = Math.min(1, lo + window);
        int k = 24;
        double best = lo, bd = Double.MAX_VALUE;
        for (int i = 0; i <= k; i++) {
            double tt = lo + (hi - lo) * i / k;
            double d = curve.point(tt).minus(pos).normSq();
            if (d < bd) { bd = d; best = tt; }
        }
        double step = (hi - lo) / k;
        double a = Math.max(lo, best - step), b = Math.min(hi, best + step);
        for (int i = 0; i < 25; i++) {
            double m1 = a + (b - a) / 3, m2 = b - (b - a) / 3;
            if (curve.point(m1).minus(pos).normSq() < curve.point(m2).minus(pos).normSq()) b = m2; else a = m1;
        }
        return (a + b) / 2;
    }

    public double headingAt(double t) {
        switch (headingRule) {
            case HOLD: return startHeading;
            case LINEAR: return startHeading + MathUtil.wrap(endHeading - startHeading) * MathUtil.clamp(t, 0, 1);
            default: {
                Vec tan = curve.tangent(t);
                return Math.atan2(tan.y, tan.x) + (headingRule == Heading.REVERSED ? Math.PI : 0);
            }
        }
    }
}
