package com.flowpath;

/**
 * Field pose. Origin = field corner, +x right, +y up, heading 0 = facing +x, counter-clockwise positive.
 * Create with degrees: Pose.of(24, 72, 90). The raw constructor takes RADIANS.
 */
public final class Pose {
    public final double x, y, heading; // heading in radians

    public Pose(double x, double y, double headingRad) { this.x = x; this.y = y; this.heading = headingRad; }

    public static Pose of(double x, double y, double headingDeg) { return new Pose(x, y, Math.toRadians(headingDeg)); }
    public static Pose of(double x, double y) { return new Pose(x, y, 0); }

    public Vec vec() { return new Vec(x, y); }
    public double headingDeg() { return Math.toDegrees(heading); }
    public double distanceTo(Pose o) { return Math.hypot(o.x - x, o.y - y); }

    @Override public String toString() { return String.format("(%.2f, %.2f, %.1f deg)", x, y, headingDeg()); }
}
