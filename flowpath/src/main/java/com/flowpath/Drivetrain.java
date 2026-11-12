package com.flowpath;

/** Anything Flow can drive. Robot-centric forward / left / turn(CCW), each in [-1, 1]. */
public interface Drivetrain {
    void drive(double forward, double left, double turn);
    double[] powers(); // last commanded motor powers, shown on the dashboard

    /** true = can move sideways (mecanum / swerve). false = tank: the "left" command is ignored. */
    default boolean holonomic() { return true; }
}
