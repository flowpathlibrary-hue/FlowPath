package com.flowpath;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Odometry from the DRIVE motors' own encoders (no dead wheels, no extra sensors).
 * Works for mecanum (4 wheels) and tank (any number of motors per side).
 * Less accurate than dead wheels or a Pinpoint because wheels slip, and heading drifts the most.
 *
 * Use through the builder (it already knows your motor names):
 *   Flow.tank(hw)....driveEncoders(ticksPerInch, trackWidth)
 *   Flow.mecanum(hw)....driveEncoders(ticksPerInch, trackWidth, wheelBase)
 * Motor directions are read from the motors themselves, so make sure every wheel counts UP when it drives the robot forward
 * (the drivetrain already sets this; use invertMotors / invertSides if a wheel is wrong).
 */
public final class DriveLocalizer implements Localizer {
    private final boolean mecanum;
    private final DcMotorEx[] a; // mecanum: {fl, fr, bl, br}; tank: left side motors
    private final DcMotorEx[] b; // tank: right side motors
    private final double ticksPerInch, trackWidth, wheelBase;
    private double lateralScale = 1.0;
    private double[] last;
    private boolean init;
    private Pose pose = new Pose(0, 0, 0), vel = new Pose(0, 0, 0);
    private long lastNs;

    private DriveLocalizer(boolean mecanum, DcMotorEx[] a, DcMotorEx[] b, double ticksPerInch, double trackWidth, double wheelBase) {
        this.mecanum = mecanum; this.a = a; this.b = b;
        this.ticksPerInch = ticksPerInch; this.trackWidth = trackWidth; this.wheelBase = wheelBase;
    }

    /**
     * @param trackWidth distance between the left and right wheels (in)
     * @param wheelBase  distance between the front and back wheels (in)
     */
    public static DriveLocalizer mecanum(HardwareMap hw, String fl, String fr, String bl, String br,
                                         double ticksPerInch, double trackWidth, double wheelBase) {
        DcMotorEx[] m = {hw.get(DcMotorEx.class, fl), hw.get(DcMotorEx.class, fr),
                hw.get(DcMotorEx.class, bl), hw.get(DcMotorEx.class, br)};
        return new DriveLocalizer(true, m, null, ticksPerInch, trackWidth, wheelBase);
    }

    /** @param trackWidth distance between the left and right wheels (in) */
    public static DriveLocalizer tank(HardwareMap hw, String[] left, String[] right, double ticksPerInch, double trackWidth) {
        DcMotorEx[] l = new DcMotorEx[left.length], r = new DcMotorEx[right.length];
        for (int i = 0; i < left.length; i++) l[i] = hw.get(DcMotorEx.class, left[i]);
        for (int i = 0; i < right.length; i++) r[i] = hw.get(DcMotorEx.class, right[i]);
        return new DriveLocalizer(false, l, r, ticksPerInch, trackWidth, 0);
    }

    /** Helper: ticks per inch from the motor's ticks per revolution (at the wheel) and the wheel diameter in inches. */
    public static double ticksPerInch(double ticksPerRev, double wheelDiameterIn) {
        return ticksPerRev / (Math.PI * wheelDiameterIn);
    }

    /**
     * Mecanum only: mecanum wheels over-report sideways travel. Push the robot 48 in sideways by hand;
     * lateralScale = (distance the localizer says) / 48. Typical 1.1 to 1.3. Default 1.0.
     */
    public DriveLocalizer lateralScale(double s) { lateralScale = s; return this; }

    private double[] read() {
        if (mecanum) {
            return new double[]{a[0].getCurrentPosition(), a[1].getCurrentPosition(),
                    a[2].getCurrentPosition(), a[3].getCurrentPosition()};
        }
        double l = 0, r = 0;
        for (DcMotorEx m : a) l += m.getCurrentPosition();
        for (DcMotorEx m : b) r += m.getCurrentPosition();
        return new double[]{l / a.length, r / b.length};
    }

    @Override
    public void update() {
        double[] c = read();
        long now = System.nanoTime();
        if (!init) { last = c; lastNs = now; init = true; return; }
        double[] d = new double[c.length];
        for (int i = 0; i < c.length; i++) d[i] = (c[i] - last[i]) / ticksPerInch;
        last = c;
        double dt = Math.max(1e-3, (now - lastNs) / 1e9);
        lastNs = now;

        double dx, dy, dTheta; // robot frame: forward, left, CCW
        if (mecanum) { // wheel order fl, fr, bl, br (same mixing as MecanumDrive)
            dx = (d[0] + d[1] + d[2] + d[3]) / 4.0;
            dy = (-d[0] + d[1] + d[2] - d[3]) / 4.0 / lateralScale;
            dTheta = (-d[0] + d[1] - d[2] + d[3]) / (2.0 * (trackWidth + wheelBase));
        } else {
            dx = (d[0] + d[1]) / 2.0;
            dy = 0;
            dTheta = (d[1] - d[0]) / trackWidth;
        }

        double s, cc; // pose exponential (arc) instead of a straight-line approximation
        if (Math.abs(dTheta) < 1e-6) { s = 1 - dTheta * dTheta / 6; cc = dTheta / 2; }
        else { s = Math.sin(dTheta) / dTheta; cc = (1 - Math.cos(dTheta)) / dTheta; }
        double rx = s * dx - cc * dy, ry = cc * dx + s * dy;

        double cosH = Math.cos(pose.heading), sinH = Math.sin(pose.heading);
        double fx = rx * cosH - ry * sinH, fy = rx * sinH + ry * cosH;
        pose = new Pose(pose.x + fx, pose.y + fy, MathUtil.wrap(pose.heading + dTheta));

        double k = 0.6; // light low-pass on velocity
        vel = new Pose(vel.x + k * (fx / dt - vel.x), vel.y + k * (fy / dt - vel.y),
                vel.heading + k * (dTheta / dt - vel.heading));
    }

    @Override public Pose pose() { return pose; }
    @Override public Pose velocity() { return vel; }
    @Override public void setPose(Pose p) { pose = p; }
}
