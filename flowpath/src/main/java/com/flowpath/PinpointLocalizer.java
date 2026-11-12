package com.flowpath;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * goBILDA Pinpoint odometry computer. Works for mecanum and tank.
 *
 * <pre>
 * Flow.mecanum(hw)....pinpoint("pinpoint", -3.5, 6.0)      // quick
 * Flow.mecanum(hw)....localizer(new PinpointLocalizer(hw, "pinpoint", -3.5, 6.0).swingarmPods().reverse(false, true))
 * </pre>
 * Keep the robot still while it initialises (the Pinpoint calibrates its IMU).
 * Heading velocity is derived from the heading itself, so this works with any version of the SDK driver.
 */
public final class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver pp;
    private boolean xRev, yRev;
    private Pose pose = new Pose(0, 0, 0), vel = new Pose(0, 0, 0);
    private double lastHeading, headingVel;
    private long lastNs;
    private boolean init;

    /**
     * @param name       config name of the Pinpoint (I2C device)
     * @param xOffsetIn  how far SIDEWAYS the X pod (the one measuring forward/back) is from the robot center, in inches. Left = positive
     * @param yOffsetIn  how far FORWARD the Y pod (the one measuring strafe) is from the robot center, in inches. Forward = positive
     */
    public PinpointLocalizer(HardwareMap hw, String name, double xOffsetIn, double yOffsetIn) {
        pp = hw.get(GoBildaPinpointDriver.class, name);
        pp.setOffsets(xOffsetIn, yOffsetIn, DistanceUnit.INCH);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        applyDirections();
        pp.resetPosAndIMU();
    }

    /** Use the goBILDA 4-bar pods (this is the default). */
    public PinpointLocalizer fourBarPods() {
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        return this;
    }

    /** Use the goBILDA swingarm pods. */
    public PinpointLocalizer swingarmPods() {
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        return this;
    }

    /** Use other wheels: encoder ticks per millimetre of travel. */
    public PinpointLocalizer ticksPerMm(double ticksPerMm) {
        pp.setEncoderResolution(ticksPerMm, DistanceUnit.MM);
        return this;
    }

    /** Flip a pod until forward / strafe left both count up (true = flip). */
    public PinpointLocalizer reverse(boolean x, boolean y) {
        xRev = x; yRev = y;
        applyDirections();
        return this;
    }

    private void applyDirections() {
        pp.setEncoderDirections(
                xRev ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                yRev ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    @Override
    public void update() {
        pp.update();
        double h = pp.getHeading(AngleUnit.RADIANS);
        long now = System.nanoTime();
        if (init) {
            double dt = Math.max(1e-3, (now - lastNs) / 1e9);
            headingVel += 0.6 * (MathUtil.wrap(h - lastHeading) / dt - headingVel); // light low-pass
        } else {
            headingVel = 0;
            init = true;
        }
        lastHeading = h;
        lastNs = now;
        pose = new Pose(pp.getPosX(DistanceUnit.INCH), pp.getPosY(DistanceUnit.INCH), h);
        vel = new Pose(pp.getVelX(DistanceUnit.INCH), pp.getVelY(DistanceUnit.INCH), headingVel);
    }

    @Override public Pose pose() { return pose; }
    @Override public Pose velocity() { return vel; }

    @Override
    public void setPose(Pose p) {
        pp.setPosition(new Pose2D(DistanceUnit.INCH, p.x, p.y, AngleUnit.RADIANS, p.heading));
        pose = p;
        init = false; // restart the heading-rate filter after a jump
    }
}
