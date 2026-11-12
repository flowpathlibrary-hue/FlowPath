package com.flowpath;

/** Bezier curve of any degree: 2 points = line, 3 = quadratic, 4 = cubic, and so on. */
public final class Curve {
    private final Vec[] p, d1, d2; // d2 == null for lines

    public Curve(Vec... points) {
        if (points.length < 2) throw new IllegalArgumentException("A curve needs at least 2 points");
        p = points.clone();
        d1 = derivative(p);
        d2 = d1.length >= 2 ? derivative(d1) : null;
    }

    private static Vec[] derivative(Vec[] q) {
        int n = q.length - 1;
        Vec[] r = new Vec[n];
        for (int i = 0; i < n; i++) r[i] = q[i + 1].minus(q[i]).times(n);
        return r;
    }

    private static Vec eval(Vec[] q, double t) {
        int n = q.length;
        double[] xs = new double[n], ys = new double[n];
        for (int i = 0; i < n; i++) { xs[i] = q[i].x; ys[i] = q[i].y; }
        for (int k = 1; k < n; k++)
            for (int i = 0; i < n - k; i++) {
                xs[i] = xs[i] * (1 - t) + xs[i + 1] * t;
                ys[i] = ys[i] * (1 - t) + ys[i + 1] * t;
            }
        return new Vec(xs[0], ys[0]);
    }

    public Vec point(double t) { return eval(p, t); }
    public Vec velocity(double t) { return eval(d1, t); }
    public Vec acceleration(double t) { return d2 == null ? Vec.ZERO : eval(d2, t); }
    public Vec start() { return p[0]; }
    public Vec end() { return p[p.length - 1]; }

    /** Unit tangent; falls back to the chord direction when the derivative vanishes. */
    public Vec tangent(double t) {
        Vec d = velocity(t);
        if (d.norm() > 1e-6) return d.unit();
        Vec chord = end().minus(start());
        return chord.norm() > 1e-6 ? chord.unit() : new Vec(1, 0);
    }

    /** Signed curvature (1/inch); positive = bending left. */
    public double curvature(double t) {
        Vec a = velocity(t), b = acceleration(t);
        double den = Math.pow(a.norm(), 3);
        return den < 1e-9 ? 0 : a.cross(b) / den;
    }
}
