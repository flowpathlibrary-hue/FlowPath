package com.flowpath;

/** Immutable 2D vector / point in inches. */
public final class Vec {
    public static final Vec ZERO = new Vec(0, 0);
    public final double x, y;

    public Vec(double x, double y) { this.x = x; this.y = y; }
    public static Vec of(double x, double y) { return new Vec(x, y); }

    public Vec plus(Vec o) { return new Vec(x + o.x, y + o.y); }
    public Vec minus(Vec o) { return new Vec(x - o.x, y - o.y); }
    public Vec times(double s) { return new Vec(x * s, y * s); }
    public double dot(Vec o) { return x * o.x + y * o.y; }
    public double cross(Vec o) { return x * o.y - y * o.x; }
    public double norm() { return Math.hypot(x, y); }
    public double normSq() { return x * x + y * y; }

    public Vec unit() {
        double n = norm();
        return n < 1e-9 ? ZERO : new Vec(x / n, y / n);
    }

    @Override public String toString() { return "(" + x + ", " + y + ")"; }
}
