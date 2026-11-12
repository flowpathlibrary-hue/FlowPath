package com.flowpath;

/**
 * Every public static non-final field of the nested classes below is a live slider in the dashboard.
 * Defaults are safe starting points. Run FlowAutoTune first (measures maxSpeed / decel / strafeGain),
 * then fine tune Translation / Heading / Curve with FlowTuner.
 */
public final class Tune {
    private Tune() {}

    public static class General {
        public static double maxPower = 1.0;
        public static boolean voltageCompensation = true;
        public static double nominalVoltage = 12.5;
        public static double forwardGain = 1.0;   // multiplies forward command
        public static double strafeGain = 1.0;    // >1 if strafing is weaker than driving forward (AutoTune measures it)
    }

    /** Position PID: corrects cross-track error while following, and x/y error while holding a point. */
    public static class Translation {
        public static double kP = 0.10;           // power per inch
        public static double kI = 0.0;
        public static double kD = 0.012;
        public static double kS = 0.03;           // static friction boost while still outside tolerance (hold only)
    }

    /** Heading PID (power per radian). */
    public static class Heading {
        public static double kP = 1.6;
        public static double kI = 0.0;
        public static double kD = 0.12;
        public static double kS = 0.03;
    }

    /** Speed along the path: feedforward from measured speed + velocity PID + deceleration profile. */
    public static class Drive {
        public static double maxSpeed = 50.0;     // in/s at full power  (AutoTune measures)
        public static double decel = 40.0;        // in/s^2 braking      (AutoTune measures)
        public static double velKP = 0.02;        // power per in/s of speed error
    }

    /** Extra lateral push in curves: power = gain * v^2 * curvature. */
    public static class Curves {
        public static double centripetalGain = 0.0015;
    }

    /** When a segment / route counts as finished. */
    public static class Finish {
        public static double holdDistance = 5.0;       // in, last segment hands over to point-hold this close to the end
        public static double chainDistance = 2.0;      // in, switch to the next segment this close to the end
        public static double posTol = 0.75;            // in
        public static double headingTolDeg = 2.0;
        public static double velTol = 1.5;             // in/s
        public static double searchWindow = 0.3;       // closest-point search window (path parameter)
    }

    /** Only used when the robot is a TankDrive (Flow.tank). A tank can't strafe, so it steers by turning instead. */
    public static class Tank {
        public static double crossGain = 0.12;        // 1/in, how hard to aim back at the path (Stanley): angle = atan(crossGain * crossError)
        public static double maxCrossAngleDeg = 60;   // cap on that aiming angle
        public static double turnFF = 0.12;           // power per rad/s of path rotation (AutoTune measures: 1 / max turn rate)
        public static double arriveDist = 0.6;        // in, closer than this to a hold point: stop driving, just rotate to final heading
        public static double settleDist = 1.0;        // in, hold counts as settled inside this distance (a tank can't fix sideways error)
    }

    public static class Filter {
        public static double derivativeAlpha = 0.5;    // 1 = raw derivative, lower = smoother
        public static double integralMax = 0.3;        // max power contribution of the I term
    }
}
