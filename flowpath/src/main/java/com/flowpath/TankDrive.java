package com.flowpath;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Tank / differential drive with any number of motors per side (2, 4, 6 ...).
 * Same interface as MecanumDrive: drive(forward, left, turn). "left" is ignored, a tank cannot strafe.
 */
public final class TankDrive implements Drivetrain {
    private final DcMotorEx[] leftMotors, rightMotors;
    private final VoltageSensor battery;
    private double[] powers = new double[2]; // {left side, right side}
    private double voltage = 12.5;
    private long lastVoltNs;

    /** Defaults: left motors REVERSE, right motors FORWARD. Flip a whole side with invert(). */
    public TankDrive(HardwareMap hw, String[] left, String[] right) {
        leftMotors = new DcMotorEx[left.length];
        rightMotors = new DcMotorEx[right.length];
        for (int i = 0; i < left.length; i++) leftMotors[i] = hw.get(DcMotorEx.class, left[i]);
        for (int i = 0; i < right.length; i++) rightMotors[i] = hw.get(DcMotorEx.class, right[i]);
        battery = hw.voltageSensor.iterator().hasNext() ? hw.voltageSensor.iterator().next() : null;
        for (DcMotorEx m : leftMotors) setup(m, DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : rightMotors) setup(m, DcMotorSimple.Direction.FORWARD);
    }

    private static void setup(DcMotorEx m, DcMotorSimple.Direction dir) {
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Flip the direction of a whole side (true = flip). */
    public TankDrive invert(boolean left, boolean right) {
        if (left) for (DcMotorEx m : leftMotors) flip(m);
        if (right) for (DcMotorEx m : rightMotors) flip(m);
        return this;
    }

    private static void flip(DcMotorEx m) {
        m.setDirection(m.getDirection() == DcMotorSimple.Direction.FORWARD
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void drive(double f, double ignoredLeft, double w) {
        double l = f - w, r = f + w; // turn is CCW positive: the right side runs faster
        double scale = 1.0;
        if (Tune.General.voltageCompensation && battery != null) {
            long now = System.nanoTime();
            if (now - lastVoltNs > 250_000_000L) { voltage = battery.getVoltage(); lastVoltNs = now; }
            if (voltage > 5) scale = MathUtil.clamp(Tune.General.nominalVoltage / voltage, 0.9, 1.3);
        }
        l *= scale; r *= scale;
        double m = Math.max(1.0, Math.max(Math.abs(l), Math.abs(r)));
        powers = new double[]{l / m, r / m};
        for (DcMotorEx mo : leftMotors) mo.setPower(powers[0]);
        for (DcMotorEx mo : rightMotors) mo.setPower(powers[1]);
    }

    @Override public double[] powers() { return powers; }
    @Override public boolean holonomic() { return false; }
}
