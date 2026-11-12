package com.flowpath;

import com.flowpath.dash.Params;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * Measures the drivetrain constants that make Flow behave: top speed, strafe speed, braking deceleration.
 * Needs ~120 in of free space ahead and ~60 in to the left. Start the robot near the right-bottom of the
 * field facing +x (for example Pose.of(20, 20, 0)). Run it via the FlowAutoTune OpMode.
 */
public final class AutoTuner {
    private AutoTuner() {}

    public static final class Result {
        public double maxSpeed, strafeSpeed, decel, turnRate; // turnRate (rad/s) is measured on tank only
        @Override public String toString() {
            if (turnRate > 0) return String.format("maxSpeed=%.1f in/s  turnRate=%.2f rad/s  decel=%.1f in/s^2", maxSpeed, turnRate, decel);
            return String.format("maxSpeed=%.1f in/s  strafeSpeed=%.1f in/s  decel=%.1f in/s^2", maxSpeed, strafeSpeed, decel);
        }
    }

    private static final double DT = 0.02;
    private static final double RUN_DISTANCE = 60;

    /** Blocking measurement; pass opModeIsActive as `active`. Applies the values to Tune and saves them. */
    public static Result measure(Flow flow, BooleanSupplier active, Consumer<String> log) throws InterruptedException {
        Result r = new Result();
        double[] fwd = runAndBrake(flow, 1, 0, active, log, "forward");
        r.maxSpeed = fwd[0]; r.decel = fwd[1];
        if (flow.drivetrain().holonomic()) {
            double[] side = runAndBrake(flow, 0, 1, active, log, "strafe left");
            r.strafeSpeed = side[0];
        } else {
            r.turnRate = measureTurnRate(flow, active, log); // tank: can't strafe, measure spin speed instead
        }

        if (r.maxSpeed > 5) {
            Tune.Drive.maxSpeed = r.maxSpeed;
            Tune.Drive.decel = Math.max(10, r.decel * 0.8); // 20% margin so the robot starts braking a bit early
            if (r.turnRate > 0.5) Tune.Tank.turnFF = MathUtil.clamp(1.0 / r.turnRate, 0.02, 1.0);
            if (r.strafeSpeed > 5) Tune.General.strafeGain = MathUtil.clamp(r.maxSpeed / r.strafeSpeed, 1.0, 2.5);
            log.accept("saved=" + Params.save() + "  " + r);
        } else log.accept("Measurement failed (robot did not move). Check motor directions and the localizer.");
        return r;
    }

    /** Tank: spin in place at full power and return the top rotation speed (rad/s). */
    private static double measureTurnRate(Flow f, BooleanSupplier active, Consumer<String> log) throws InterruptedException {
        log.accept("Running spin...");
        double top = 0, elapsed = 0;
        while (active.getAsBoolean() && elapsed < 1.5) {
            f.drivetrain().drive(0, 0, 1);
            Thread.sleep((long) (DT * 1000)); elapsed += DT;
            f.sense();
            if (elapsed > 0.5) top = Math.max(top, Math.abs(f.velocity().heading));
        }
        f.drivetrain().drive(0, 0, 0);
        Thread.sleep(400);
        log.accept(String.format("  spin: %.2f rad/s", top));
        return top;
    }

    /** @return {top speed, deceleration} for one direction. */
    private static double[] runAndBrake(Flow f, double fwd, double left, BooleanSupplier active, Consumer<String> log, String name)
            throws InterruptedException {
        log.accept("Running " + name + "...");
        f.sense();
        Vec p0 = f.pose().vec();
        double top = 0, elapsed = 0;
        while (active.getAsBoolean() && elapsed < 2.5) {
            f.drivetrain().drive(fwd, left, 0);
            Thread.sleep((long) (DT * 1000)); elapsed += DT;
            f.sense();
            double speed = f.velocity().vec().norm();
            if (elapsed > 0.6) top = Math.max(top, speed);
            if (f.pose().vec().minus(p0).norm() > RUN_DISTANCE) break;
        }
        double v0 = f.velocity().vec().norm();
        Vec pBrake = f.pose().vec();
        f.drivetrain().drive(0, 0, 0);
        elapsed = 0;
        while (active.getAsBoolean() && elapsed < 2.5) {
            Thread.sleep((long) (DT * 1000)); elapsed += DT;
            f.sense();
            if (f.velocity().vec().norm() < 2.0) break;
        }
        double stopDist = Math.max(0.5, f.pose().vec().minus(pBrake).norm());
        double decel = v0 * v0 / (2 * stopDist);
        log.accept(String.format("  %s: top %.1f in/s, decel %.1f in/s^2", name, top, decel));
        return new double[]{top, decel};
    }
}
