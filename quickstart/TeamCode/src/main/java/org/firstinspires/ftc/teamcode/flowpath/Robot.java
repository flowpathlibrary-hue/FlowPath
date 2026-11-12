package org.firstinspires.ftc.teamcode.flowpath;

import com.flowpath.Flow;
import com.flowpath.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Describe YOUR robot once; every OpMode calls Robot.create(hardwareMap).
 *
 * TANK DRIVE: replace Flow.mecanum(hw).motors(...) with
 *     Flow.tank(hw)
 *         .leftMotors("frontLeft", "backLeft")        // any number of motors per side
 *         .rightMotors("frontRight", "backRight")
 *         // .invertSides(false, false)                // flip a whole side if it drives backwards
 * everything else (.threeWheel / .localizer / .start, routes, Ivy commands, tuners) stays the same.
 */
public final class Robot {
    private Robot() {}

    public static Flow create(HardwareMap hw) {
        return Flow.mecanum(hw)
                .motors("frontLeft", "frontRight", "backLeft", "backRight")
                .threeWheel("frontLeft", "frontRight", "backRight",   // encoder ports: left, right, strafe
                        1982,    // ticks per inch   (2000 tick/rev, 48 mm wheel: 2000 / (PI * 1.89))
                        13.0,    // track width, in  (distance between the two parallel wheels)
                        -5.0)    // strafe wheel offset, in (+ = in front of robot center)
                .start(Pose.of(0, 0, 0))
                // .invertMotors(false, false, false, false)   // flip wheels that spin the wrong way
                // .reverseEncoders(false, false, false)       // flip encoders that count backwards
                // --- pilih SATU sumber odometri (ganti .threeWheel di atas) ---
                // .pinpoint("pinpoint", -3.5, 6.0)            // goBILDA Pinpoint: x pod offset (kiri +), y pod offset (depan +), inch
                // .driveEncoders(336.8, 13.0)                 // encoder motor roda, TANK: ticks/inch, track width
                // .driveEncoders(336.8, 13.0, 12.0)           // encoder motor roda, MECANUM: + wheel base (jarak depan-belakang)
                // .driveLateralScale(1.15)                    // mecanum drive encoder: koreksi strafe (1.0 = tanpa)
                // .localizer(Localizer.custom(...))           // odometri lain (OTOS, dll)
                .build();   // also starts the dashboard at http://192.168.43.1:8099
    }
}
