package com.flowpath;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Three dead-wheel odometry (two parallel + one perpendicular), encoder-only heading.
 * Names are the config names of the motor ports the encoders are plugged into.
 */
public final class ThreeWheelLocalizer implements Localizer {
    private final DcMotorEx left, right, strafe;
    private final double ticksPerInch, trackWidth, strafeOffset;
    private int sl = 1, sr = 1, ss = 1;
    private int lastL, lastR, lastS;
    private boolean init;
    private Pose pose = new Pose(0, 0, 0), vel = new Pose(0, 0, 0);
    private long lastNs;

    /**
     * @param trackWidth   distance between the two parallel wheels (in)
     * @param strafeOffset how far the perpendicular wheel is IN FRONT of the robot center (in, negative = behind)
     */
    public ThreeWheelLocalizer(HardwareMap hw, String leftEnc, String rightEnc, String strafeEnc,
                               double ticksPerInch, double trackWidth, double strafeOffset) {
        left = hw.get(DcMotorEx.class, leftEnc);
        right = hw.get(DcMotorEx.class, rightEnc);
        strafe = hw.get(DcMotorEx.class, strafeEnc);
        this.ticksPerInch = ticksPerInch;
        this.trackWidth = trackWidth;
        this.strafeOffset = strafeOffset;
    }

    /** Flip an encoder until driving forward / strafing left / turning CCW all count up. */
    public ThreeWheelLocalizer reverse(boolean l, boolean r, boolean s) {
        sl = l ? -1 : 1; sr = r ? -1 : 1; ss = s ? -1 : 1;
        return this;
    }

    @Override
    public void update() {
        int cl = sl * left.getCurrentPosition(), cr = sr * right.getCurrentPosition(), cs = ss * strafe.getCurrentPosition();
        long now = System.nanoTime();
        if (!init) { lastL = cl; lastR = cr; lastS = cs; lastNs = now; init = true; return; }

        double dL = (cl - lastL) / ticksPerInch, dR = (cr - lastR) / ticksPerInch, dS = (cs - lastS) / ticksPerInch;
        lastL = cl; lastR = cr; lastS = cs;
        double dt = Math.max(1e-3, (now - lastNs) / 1e9);
        lastNs = now;

        double dTheta = (dR - dL) / trackWidth;
        double dx = (dL + dR) / 2.0;
        double dy = dS - strafeOffset * dTheta;

        double s, c; // pose exponential (arc) instead of a straight-line approximation
        if (Math.abs(dTheta) < 1e-6) { s = 1 - dTheta * dTheta / 6; c = dTheta / 2; }
        else { s = Math.sin(dTheta) / dTheta; c = (1 - Math.cos(dTheta)) / dTheta; }
        double rx = s * dx - c * dy, ry = c * dx + s * dy;

        double cosH = Math.cos(pose.heading), sinH = Math.sin(pose.heading);
        double fx = rx * cosH - ry * sinH, fy = rx * sinH + ry * cosH;
        pose = new Pose(pose.x + fx, pose.y + fy, MathUtil.wrap(pose.heading + dTheta));

        double a = 0.6; // light low-pass on velocity
        vel = new Pose(vel.x + a * (fx / dt - vel.x), vel.y + a * (fy / dt - vel.y),
                vel.heading + a * (dTheta / dt - vel.heading));
    }

    @Override public Pose pose() { return pose; }
    @Override public Pose velocity() { return vel; }
    @Override public void setPose(Pose p) { pose = p; }
}
