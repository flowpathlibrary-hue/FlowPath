package com.flowpath;

import static com.flowpath.MathUtil.clamp;
import static com.flowpath.MathUtil.wrap;

import com.flowpath.dash.Dash;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collections;

/**
 * The robot controller. Call update() once per loop.
 *
 * ROUTE : speed along the path = feedforward(profile) + velocity PID. Cross-track PID + centripetal
 *         feedforward steer sideways, a heading PID turns. Only the last segment brakes (decel profile).
 * HOLD  : x/y translational PID + heading PID to a pose (end of a route, hold(), turnTo()).
 * MANUAL: gamepad driving, robot- or field-centric.
 *
 * <pre>
 * Flow flow = Flow.mecanum(hardwareMap)
 *     .motors("fl", "fr", "bl", "br")
 *     .threeWheel("lEnc", "rEnc", "sEnc", 1982, 13.0, -5.0)
 *     .start(Pose.of(24, 72, 0))
 *     .build();
 * flow.run(Route.from(flow.pose()).lineTo(Pose.of(60, 72, 90)).build());
 * while (opModeIsActive()) flow.update();
 * </pre>
 */
public final class Flow {
    public enum Mode { IDLE, ROUTE, HOLD, MANUAL }

    private final Drivetrain drive;
    private final Localizer localizer;
    private final PID crossPid = new PID(), headPid = new PID(), holdX = new PID(), holdY = new PID();

    private Mode mode = Mode.IDLE;
    private Route route;
    private int segIdx;
    private Segment seg;
    private double t;
    private boolean[] fired = new boolean[0];
    private Pose holdTarget = new Pose(0, 0, 0);
    private double curMaxPower = 1.0;
    private boolean settled = true;
    private long lastNs;

    private Pose pose = new Pose(0, 0, 0), vel = new Pose(0, 0, 0);
    private double cross, headErr, remaining, targetHeading;
    private Vec closest = Vec.ZERO;
    private double manF, manL, manW;
    private boolean manField;
    private boolean tankFinal; // tank hold: close enough, only rotating to the final heading

    public Flow(Drivetrain drive, Localizer localizer) {
        this.drive = drive;
        this.localizer = localizer;
    }

    // ------------------------------------------------------------------ builder

    /** Start describing a mecanum robot. */
    public static Builder mecanum(HardwareMap hw) { return new Builder(hw); }

    /** Start describing a tank / differential robot. Use leftMotors(...) and rightMotors(...) instead of motors(...). */
    public static Builder tank(HardwareMap hw) { Builder b = new Builder(hw); b.tank = true; return b; }

    public static final class Builder {
        private final HardwareMap hw;
        private String[] motors;
        private boolean[] invert = new boolean[4];
        private Localizer localizer;
        private Pose start = new Pose(0, 0, 0);
        private boolean dashboard = true;
        private Class<?>[] tunables = new Class<?>[0];
        private String[] enc;
        private double tpi, track, offset;
        private boolean[] encRev = new boolean[3];
        private boolean tank;
        private boolean driveOdo;
        private double dTpi, dTrack, dBase, dLateral = 1.0;
        private String[] leftMotors, rightMotors;
        private boolean invertLeft, invertRight;

        Builder(HardwareMap hw) { this.hw = hw; }

        /** Config names: front-left, front-right, back-left, back-right. */
        public Builder motors(String fl, String fr, String bl, String br) { motors = new String[]{fl, fr, bl, br}; return this; }
        /** Tank only: config names of every motor on the left side (any number). */
        public Builder leftMotors(String... names) { leftMotors = names; return this; }
        /** Tank only: config names of every motor on the right side (any number). */
        public Builder rightMotors(String... names) { rightMotors = names; return this; }
        /** Tank only: flip a whole side (true = flip) if that side drives backwards. */
        public Builder invertSides(boolean left, boolean right) { invertLeft = left; invertRight = right; return this; }
        /** Flip motor directions (true = flip) if a wheel spins the wrong way. (mecanum) */
        public Builder invertMotors(boolean fl, boolean fr, boolean bl, boolean br) { invert = new boolean[]{fl, fr, bl, br}; return this; }
        public Builder threeWheel(String leftEnc, String rightEnc, String strafeEnc,
                                  double ticksPerInch, double trackWidth, double strafeOffset) {
            enc = new String[]{leftEnc, rightEnc, strafeEnc}; tpi = ticksPerInch; track = trackWidth; offset = strafeOffset; return this;
        }
        public Builder reverseEncoders(boolean left, boolean right, boolean strafe) { encRev = new boolean[]{left, right, strafe}; return this; }
        /** Use your own odometry (OTOS, ...). Overrides threeWheel(). For Pinpoint and drive encoders use pinpoint(...) / driveEncoders(...). */
        public Builder localizer(Localizer l) { localizer = l; return this; }
        /** goBILDA Pinpoint. xOffset: sideways offset of the forward pod (in, left +). yOffset: forward offset of the strafe pod (in, forward +). */
        public Builder pinpoint(String name, double xOffsetIn, double yOffsetIn) {
            localizer = new PinpointLocalizer(hw, name, xOffsetIn, yOffsetIn); return this;
        }
        /** Odometry from the drive motor encoders (tank: track width only). Overrides threeWheel(). */
        public Builder driveEncoders(double ticksPerInch, double trackWidth) { return driveEncoders(ticksPerInch, trackWidth, 0); }
        /** Odometry from the drive motor encoders (mecanum needs the wheel base: front-to-back wheel distance, in). */
        public Builder driveEncoders(double ticksPerInch, double trackWidth, double wheelBase) {
            driveOdo = true; dTpi = ticksPerInch; dTrack = trackWidth; dBase = wheelBase; return this;
        }
        /** Mecanum drive-encoder odometry: sideways correction, 1.0 = none (typical 1.1 to 1.3). */
        public Builder driveLateralScale(double scale) { dLateral = scale; return this; }
        public Builder start(Pose p) { start = p; return this; }
        /** Disable the web dashboard. */
        public Builder noDashboard() { dashboard = false; return this; }
        /** Extra classes whose public static double/int/boolean fields become dashboard sliders. */
        public Builder tunables(Class<?>... classes) { tunables = classes; return this; }

        public Flow build() {
            if (tank) {
                if (leftMotors == null || rightMotors == null || leftMotors.length == 0 || rightMotors.length == 0)
                    throw new IllegalStateException("Call .leftMotors(...) and .rightMotors(...)");
            } else if (motors == null) throw new IllegalStateException("Call .motors(fl, fr, bl, br)");
            for (LynxModule m : hw.getAll(LynxModule.class)) m.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            Drivetrain d = tank
                    ? new TankDrive(hw, leftMotors, rightMotors).invert(invertLeft, invertRight)
                    : new MecanumDrive(hw, motors[0], motors[1], motors[2], motors[3])
                            .invert(invert[0], invert[1], invert[2], invert[3]);
            Localizer loc = localizer;
            if (loc == null && driveOdo) {
                if (tank) loc = DriveLocalizer.tank(hw, leftMotors, rightMotors, dTpi, dTrack);
                else {
                    if (dBase <= 0) throw new IllegalStateException("Mecanum driveEncoders needs the wheel base (front-to-back distance, in)");
                    loc = DriveLocalizer.mecanum(hw, motors[0], motors[1], motors[2], motors[3], dTpi, dTrack, dBase).lateralScale(dLateral);
                }
            }
            if (loc == null) {
                if (enc == null) throw new IllegalStateException("Call .threeWheel(...), .pinpoint(...), .driveEncoders(...) or .localizer(...)");
                loc = new ThreeWheelLocalizer(hw, enc[0], enc[1], enc[2], tpi, track, offset)
                        .reverse(encRev[0], encRev[1], encRev[2]);
            }
            if (dashboard) Dash.start(tunables);
            Flow f = new Flow(d, loc);
            f.setPose(start);
            return f;
        }
    }

    // ------------------------------------------------------------------ commands

    public Pose pose() { return pose; }
    public Pose velocity() { return vel; }
    public void setPose(Pose p) { localizer.setPose(p); pose = p; }
    public Drivetrain drivetrain() { return drive; }
    public Localizer localizer() { return localizer; }

    /** Follow a route; busy() turns false once the robot has settled at the end (or is close, if holdEnd is false). */
    public void run(Route r) {
        route = r;
        Dash.routeChanged();
        resetPids();
        startSegment(0);
        mode = Mode.ROUTE;
        settled = false;
    }

    /** Move to and hold a pose. */
    public void hold(Pose target) {
        holdTarget = target;
        curMaxPower = Tune.General.maxPower;
        resetPids();
        mode = Mode.HOLD;
        settled = false;
    }

    /** Rotate in place to an absolute heading in degrees. */
    public void turnTo(double headingDeg) { hold(Pose.of(pose.x, pose.y, headingDeg)); }

    /** Cut power and go idle. */
    public void stop() { mode = Mode.IDLE; drive.drive(0, 0, 0); }

    /** Gamepad driving. strafe: left is positive; turn: counter-clockwise is positive. */
    public void manual(double forward, double strafe, double turn, boolean fieldCentric) {
        mode = Mode.MANUAL;
        manF = forward; manL = strafe; manW = turn; manField = fieldCentric;
    }

    public boolean busy() { return mode == Mode.ROUTE || (mode == Mode.HOLD && !settled); }
    public Mode mode() { return mode; }

    /** Read sensors only (no control). Used by the auto tuners. */
    public void sense() { localizer.update(); pose = localizer.pose(); vel = localizer.velocity(); }

    // ------------------------------------------------------------------ main loop

    public void update() {
        long now = System.nanoTime();
        double dt = lastNs == 0 ? 0.02 : (now - lastNs) / 1e9;
        lastNs = now;
        update(dt);
    }

    /** update() with an explicit loop time (seconds); used by the simulator tests. */
    public void update(double dt) {
        dt = clamp(dt, 0.001, 0.1);
        sense();

        crossPid.set(Tune.Translation.kP, Tune.Translation.kI, Tune.Translation.kD);
        holdX.set(Tune.Translation.kP, Tune.Translation.kI, Tune.Translation.kD);
        holdY.set(Tune.Translation.kP, Tune.Translation.kI, Tune.Translation.kD);
        headPid.set(Tune.Heading.kP, Tune.Heading.kI, Tune.Heading.kD);

        switch (mode) {
            case ROUTE: updateRoute(dt); break;
            case HOLD: updateHold(dt); break;
            case MANUAL:
                if (!drive.holonomic()) drive.drive(manF * Tune.General.forwardGain, 0, manW); // tank: always robot-centric, no strafe
                else if (manField) command(new Vec(manF, manL), manW);
                else drive.drive(manF * Tune.General.forwardGain, manL * Tune.General.strafeGain, manW);
                break;
            default: drive.drive(0, 0, 0);
        }
        Dash.publish(this);
    }

    // ------------------------------------------------------------------ route mode

    private void startSegment(int idx) {
        segIdx = idx;
        seg = route.get(idx);
        t = 0;
        fired = new boolean[seg.callbacks.size()];
        curMaxPower = seg.maxPower();
        crossPid.reset();
    }

    private void fireCallbacks(boolean all) {
        for (int i = 0; i < fired.length; i++) {
            Segment.Callback cb = seg.callbacks.get(i);
            if (!fired[i] && (all || t >= cb.t)) { fired[i] = true; cb.action.run(); }
        }
    }

    private void updateRoute(double dt) {
        Vec pos = pose.vec();
        t = seg.closestT(pos, t, Tune.Finish.searchWindow);
        fireCallbacks(false);

        closest = seg.curve.point(t);
        Vec tan = seg.curve.tangent(t);
        Vec nrm = new Vec(-tan.y, tan.x); // left normal
        Vec end = seg.curve.end();

        cross = closest.minus(pos).dot(nrm);      // > 0: path is to the robot's left
        remaining = t >= 0.999 ? end.minus(pos).dot(tan) : seg.length() - seg.lengthAt(t);
        targetHeading = seg.headingAt(t);
        headErr = wrap(targetHeading - pose.heading);
        boolean tank = !drive.holonomic();
        double tanDir = 1; // tank: +1 drive forward along the path, -1 drive backwards (faceBackwards())
        if (tank) {
            // A tank can't hold an arbitrary heading while moving: it always points along the path
            // (or backwards if the segment says faceBackwards()). The pose headings of the route only matter at the end.
            tanDir = seg.headingRule == Segment.Heading.REVERSED ? -1 : 1;
            double aim = Math.atan(Tune.Tank.crossGain * cross);
            double maxAim = Math.toRadians(Tune.Tank.maxCrossAngleDeg);
            aim = clamp(aim, -maxAim, maxAim);
            targetHeading = Math.atan2(tan.y, tan.x) + aim + (tanDir < 0 ? Math.PI : 0);
            headErr = wrap(targetHeading - pose.heading);
        }

        boolean last = segIdx == route.size() - 1;
        if (last && !route.holdEnd()) {
            if (remaining < Tune.Finish.chainDistance || t >= 0.995) {
                fireCallbacks(true);
                mode = Mode.IDLE; settled = true; drive.drive(0, 0, 0);
                return;
            }
        } else if (last) {
            if (end.minus(pos).norm() < Tune.Finish.holdDistance || t >= 0.999) {
                fireCallbacks(true);
                holdTarget = route.endPose();
                resetPids();
                mode = Mode.HOLD;
                updateHold(dt);
                return;
            }
        } else if (remaining < Tune.Finish.chainDistance || t >= 0.995) {
            fireCallbacks(true);
            startSegment(segIdx + 1);
            updateRoute(dt);
            return;
        }

        double maxP = curMaxPower;
        double vCap = Tune.Drive.maxSpeed * maxP;
        double vTarget = vCap;
        if (last && route.holdEnd()) vTarget = Math.min(vCap, Math.sqrt(2 * Tune.Drive.decel * Math.max(0, remaining)));
        double v = vel.vec().dot(tan);
        double along = vTarget / Tune.Drive.maxSpeed + Tune.Drive.velKP * (vTarget - v);

        if (tank) {
            double turnT = headPid.calculate(headErr, dt) + Tune.Tank.turnFF * v * seg.curve.curvature(t);
            turnT = clamp(turnT, -maxP, maxP);
            double alignT = Math.max(0, Math.cos(headErr)); // don't drive off while pointing the wrong way
            double allowedT = Math.max(0, maxP - Math.abs(turnT)); // turning has priority over speed
            double fwdT = clamp(along * alignT, -allowedT, allowedT) * tanDir;
            drive.drive(fwdT * Tune.General.forwardGain, 0, turnT);
            return;
        }

        double side = crossPid.calculate(cross, dt)
                + Tune.Curves.centripetalGain * v * v * seg.curve.curvature(t);
        side = clamp(side, -maxP, maxP);
        double allowed = Math.sqrt(Math.max(0, maxP * maxP - side * side)); // correcting has priority over speed
        along = clamp(along, -allowed, allowed);

        double turn = clamp(headPid.calculate(headErr, dt), -maxP, maxP);
        command(tan.times(along).plus(nrm.times(side)), turn);
    }

    // ------------------------------------------------------------------ hold mode

    private void updateHold(double dt) {
        if (!drive.holonomic()) { updateHoldTank(dt); return; }
        Vec e = holdTarget.vec().minus(pose.vec());
        headErr = wrap(holdTarget.heading - pose.heading);
        remaining = e.norm();
        closest = holdTarget.vec();
        targetHeading = holdTarget.heading;
        cross = 0;

        Vec cmd = new Vec(holdX.calculate(e.x, dt), holdY.calculate(e.y, dt));
        if (remaining > Tune.Finish.posTol) cmd = cmd.plus(e.unit().times(Tune.Translation.kS));
        if (cmd.norm() > curMaxPower) cmd = cmd.unit().times(curMaxPower);

        double tolRad = Math.toRadians(Tune.Finish.headingTolDeg);
        double turn = headPid.calculate(headErr, dt);
        if (Math.abs(headErr) > tolRad) turn += Math.signum(headErr) * Tune.Heading.kS;
        turn = clamp(turn, -curMaxPower, curMaxPower);
        command(cmd, turn);

        settled = remaining < Tune.Finish.posTol
                && Math.abs(headErr) < tolRad
                && vel.vec().norm() < Tune.Finish.velTol;
    }

    /**
     * Tank version of hold: turn toward the point and drive to it (backwards if that is shorter),
     * then rotate in place to the final heading once close.
     */
    private void updateHoldTank(double dt) {
        Vec e = holdTarget.vec().minus(pose.vec());
        double dist = e.norm();
        closest = holdTarget.vec();
        cross = 0;
        remaining = dist;
        tankFinal = tankFinal ? dist < 2 * Tune.Tank.arriveDist : dist < Tune.Tank.arriveDist; // hysteresis

        double tolRad = Math.toRadians(Tune.Finish.headingTolDeg);
        double turn, fwd;
        if (tankFinal) {
            targetHeading = holdTarget.heading;
            headErr = wrap(targetHeading - pose.heading);
            turn = headPid.calculate(headErr, dt);
            if (Math.abs(headErr) > tolRad) turn += Math.signum(headErr) * Tune.Heading.kS;
            turn = clamp(turn, -curMaxPower, curMaxPower);
            fwd = 0;
            holdX.reset();
        } else {
            double bearing = Math.atan2(e.y, e.x);
            double err = wrap(bearing - pose.heading);
            double dir = 1;
            if (Math.abs(err) > Math.PI / 2) { err = wrap(err + Math.PI); dir = -1; } // cheaper to reverse
            targetHeading = bearing + (dir < 0 ? Math.PI : 0);
            headErr = err;
            turn = clamp(headPid.calculate(err, dt), -curMaxPower, curMaxPower);
            double f = holdX.calculate(dist, dt) + Tune.Translation.kS;
            f *= Math.max(0, Math.cos(err));
            double allowed = Math.max(0, curMaxPower - Math.abs(turn));
            fwd = clamp(f, 0, allowed) * dir;
        }
        drive.drive(fwd * Tune.General.forwardGain, 0, turn);

        settled = dist < Tune.Tank.settleDist
                && Math.abs(wrap(holdTarget.heading - pose.heading)) < tolRad
                && vel.vec().norm() < Tune.Finish.velTol;
        if (!tankFinal) settled = false;
    }

    // ------------------------------------------------------------------ helpers

    /** Field-frame translation command -> robot-frame drivetrain command. */
    private void command(Vec field, double turn) {
        double c = Math.cos(pose.heading), s = Math.sin(pose.heading);
        double fwd = field.x * c + field.y * s;
        double left = -field.x * s + field.y * c;
        drive.drive(fwd * Tune.General.forwardGain, left * Tune.General.strafeGain, turn);
    }

    private void resetPids() { crossPid.reset(); headPid.reset(); holdX.reset(); holdY.reset(); tankFinal = false; }

    // ------------------------------------------------------------------ read-only state (dashboard / telemetry)

    public Route route() { return route; }
    public int segmentIndex() { return segIdx; }
    public double progress() { return t; }
    public Vec closestPoint() { return closest; }
    public double targetHeading() { return targetHeading; }
    public double crossError() { return cross; }
    public double headingError() { return headErr; }
    public double remaining() { return remaining; }
    public double[] motorPowers() { return drive.powers(); }
}
