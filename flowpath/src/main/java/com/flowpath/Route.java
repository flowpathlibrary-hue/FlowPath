package com.flowpath;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** An ordered list of segments followed back-to-back. Only the last segment brakes (if holdEnd). */
public final class Route {
    private final List<Segment> segments;
    private final boolean holdEnd;
    private final Pose endPose;

    Route(List<Segment> segments, boolean holdEnd, Pose endPose) {
        this.segments = Collections.unmodifiableList(new ArrayList<>(segments));
        this.holdEnd = holdEnd;
        this.endPose = endPose;
    }

    /** Start describing a route from a pose. Each lineTo / curveTo continues from the previous end. */
    public static RouteBuilder from(Pose start) { return new RouteBuilder(start); }

    public int size() { return segments.size(); }
    public Segment get(int i) { return segments.get(i); }
    public List<Segment> segments() { return segments; }
    public boolean holdEnd() { return holdEnd; }
    public Pose endPose() { return endPose; }
}
