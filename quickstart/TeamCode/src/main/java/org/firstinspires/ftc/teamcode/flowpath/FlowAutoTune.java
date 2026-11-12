package org.firstinspires.ftc.teamcode.flowpath;

import com.flowpath.AutoTuner;
import com.flowpath.Flow;
import com.flowpath.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Step 1 of tuning. Put the robot near the right-bottom of the field facing +x with ~120 in free ahead and
 * ~60 in free to its left, press START. It drives forward and strafes left at full power, brakes, and
 * measures maxSpeed / strafeGain / decel, then saves them (visible and editable on the dashboard).
 */
@TeleOp(name = "Flow AutoTune (speed + braking)", group = "FlowPath")
public class FlowAutoTune extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Flow flow = Robot.create(hardwareMap);
        flow.setPose(Pose.of(20, 20, 0));
        telemetry.addLine("Clear ~120 in ahead and ~60 in left, then press START");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) return;

        AutoTuner.Result r = AutoTuner.measure(flow, this::opModeIsActive, s -> { telemetry.addLine(s); telemetry.update(); });
        flow.stop();
        telemetry.addLine("DONE: " + r);
        telemetry.update();
        while (opModeIsActive()) sleep(50);
    }
}
