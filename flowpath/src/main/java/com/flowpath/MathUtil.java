package com.flowpath;

public final class MathUtil {
    private MathUtil() {}

    /** Wrap an angle (radians) to (-PI, PI]. */
    public static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    public static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
