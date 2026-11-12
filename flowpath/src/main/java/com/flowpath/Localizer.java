package com.flowpath;

import java.util.function.Consumer;
import java.util.function.Supplier;

public interface Localizer {
    /** Called once per loop. */
    void update();
    /** Field pose (inches, radians). */
    Pose pose();
    /** Field velocity (in/s, rad/s) in Pose.x / y / heading. */
    Pose velocity();
    void setPose(Pose pose);

    /**
     * Wrap any odometry source (goBILDA Pinpoint, OTOS, your own) in 4 lambdas, e.g. for Pinpoint:
     * <pre>
     * Localizer.custom(pinpoint::update,
     *     () -> new Pose(pinpoint.getPosX(INCH), pinpoint.getPosY(INCH), pinpoint.getHeading(RADIANS)),
     *     () -> new Pose(pinpoint.getVelX(INCH), pinpoint.getVelY(INCH), pinpoint.getHeadingVelocity(...)),
     *     p -> pinpoint.setPosition(new Pose2D(INCH, p.x, p.y, RADIANS, p.heading)));
     * </pre>
     */
    static Localizer custom(Runnable update, Supplier<Pose> pose, Supplier<Pose> velocity, Consumer<Pose> setPose) {
        return new Localizer() {
            @Override public void update() { update.run(); }
            @Override public Pose pose() { return pose.get(); }
            @Override public Pose velocity() { return velocity.get(); }
            @Override public void setPose(Pose p) { setPose.accept(p); }
        };
    }
}
