package com.flowpath;

import java.util.ArrayList;
import java.util.List;

/**
 * Fluent route description. Every segment starts where the previous one ended.
 * By default the heading turns linearly from the previous pose heading to the target pose heading.
 *
 * <pre>
 * Route r = Route.from(Pose.of(24, 72, 0))
 *     .lineTo(Pose.of(60, 100, 135))
 *     .curveTo(Pose.of(48, 36, 270), Vec.of(70, 60))   // Bezier with one control point
 *         .maxPower(0.7)
 *         .at(0.5, () -> intake.run())                 // callback at 50% of this segment
 *     .lineTo(Pose.of(60, 100, 135))
 *         .faceTangent()
 *     .build();
 * </pre>
 * Modifiers (faceTangent, maxPower, at, ...) apply to the segment added last.
 */
public final class RouteBuilder {
    private final List<Segment> segs = new ArrayList<>();
    private Pose cursor;
    private Segment cur;
    private boolean holdEnd = true;

    RouteBuilder(Pose start) { cursor = start; }

    public RouteBuilder lineTo(Pose end) { return add(new Curve(cursor.vec(), end.vec()), end); }
    public RouteBuilder lineTo(double x, double y, double headingDeg) { return lineTo(Pose.of(x, y, headingDeg)); }

    /** Bezier to end through any number of control points (1 control = quadratic, 2 = cubic, ...). */
    public RouteBuilder curveTo(Pose end, Vec... controls) {
        Vec[] pts = new Vec[controls.length + 2];
        pts[0] = cursor.vec();
        System.arraycopy(controls, 0, pts, 1, controls.length);
        pts[pts.length - 1] = end.vec();
        return add(new Curve(pts), end);
    }

    private RouteBuilder add(Curve c, Pose end) {
        cur = new Segment(c, cursor.heading, end.heading);
        segs.add(cur);
        cursor = end;
        return this;
    }

    /** Point the robot along the direction of travel. */
    public RouteBuilder faceTangent() { need(); cur.headingRule = Segment.Heading.TANGENT; return this; }
    /** Drive the segment backwards (robot's back faces the direction of travel). */
    public RouteBuilder faceBackwards() { need(); cur.headingRule = Segment.Heading.REVERSED; return this; }
    /** Keep a fixed heading for the whole segment. */
    public RouteBuilder holdHeading(double deg) {
        need(); cur.headingRule = Segment.Heading.HOLD; cur.startHeading = Math.toRadians(deg); return this;
    }
    /** Cap motor power on this segment. */
    public RouteBuilder maxPower(double p) { need(); cur.maxPower = p; return this; }
    /** Run once when the robot passes parameter t (0..1) of the current segment. */
    public RouteBuilder at(double t, Runnable action) { need(); cur.callbacks.add(new Segment.Callback(t, action)); return this; }
    /** false = don't brake/hold at the end, finish as soon as the robot is close (faster autos). */
    public RouteBuilder holdEnd(boolean hold) { holdEnd = hold; return this; }

    public Route build() { need(); return new Route(segs, holdEnd, cursor); }

    private void need() { if (cur == null) throw new IllegalStateException("Add lineTo(...) or curveTo(...) first"); }
}
