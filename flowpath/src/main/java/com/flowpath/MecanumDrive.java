package com.flowpath;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public final class MecanumDrive implements Drivetrain {
    private final DcMotorEx fl, fr, bl, br;
    private final VoltageSensor battery;
    private double[] powers = new double[4];
    private double voltage = 12.5;
    private long lastVoltNs;

    /** Defaults: left motors REVERSE, right motors FORWARD. Flip any with invert(). */
    public MecanumDrive(HardwareMap hw, String fl, String fr, String bl, String br) {
        this.fl = hw.get(DcMotorEx.class, fl);
        this.fr = hw.get(DcMotorEx.class, fr);
        this.bl = hw.get(DcMotorEx.class, bl);
        this.br = hw.get(DcMotorEx.class, br);
        battery = hw.voltageSensor.iterator().hasNext() ? hw.voltageSensor.iterator().next() : null;
        this.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.bl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.fr.setDirection(DcMotorSimple.Direction.FORWARD);
        this.br.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{this.fl, this.fr, this.bl, this.br}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /** Flip the direction of individual motors (true = flip). */
    public MecanumDrive invert(boolean fl, boolean fr, boolean bl, boolean br) {
        flip(this.fl, fl); flip(this.fr, fr); flip(this.bl, bl); flip(this.br, br);
        return this;
    }

    private static void flip(DcMotorEx m, boolean f) {
        if (f) m.setDirection(m.getDirection() == DcMotorSimple.Direction.FORWARD
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void drive(double f, double l, double w) {
        double a = f - l - w, b = f + l + w, c = f + l - w, d = f - l + w; // fl fr bl br
        double scale = 1.0;
        if (Tune.General.voltageCompensation && battery != null) {
            long now = System.nanoTime();
            if (now - lastVoltNs > 250_000_000L) { voltage = battery.getVoltage(); lastVoltNs = now; }
            if (voltage > 5) scale = MathUtil.clamp(Tune.General.nominalVoltage / voltage, 0.9, 1.3);
        }
        a *= scale; b *= scale; c *= scale; d *= scale;
        double m = Math.max(1.0, Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d))));
        powers = new double[]{a / m, b / m, c / m, d / m};
        fl.setPower(powers[0]); fr.setPower(powers[1]); bl.setPower(powers[2]); br.setPower(powers[3]);
    }

    @Override public double[] powers() { return powers; }
}
