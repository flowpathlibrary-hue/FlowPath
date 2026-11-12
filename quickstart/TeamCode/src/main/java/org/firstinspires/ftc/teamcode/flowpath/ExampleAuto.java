package org.firstinspires.ftc.teamcode.flowpath;

import static com.flowpath.ivy.FlowCommands.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import com.flowpath.Flow;
import com.flowpath.Pose;
import com.flowpath.Route;
import com.flowpath.Vec;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Example autonomous: FlowPath routes + Ivy commands. Draw your own routes in the dashboard Planner tab. */
@Autonomous(name = "Flow Auto Example (Ivy)", group = "FlowPath")
public class ExampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Scheduler.reset(); // Scheduler is static: clear leftovers from the previous OpMode

        Flow flow = Robot.create(hardwareMap);
        Pose start = Pose.of(24, 72, 0), score = Pose.of(60, 100, 135), pickup = Pose.of(48, 36, 270);
        flow.setPose(start);

        Route toScore = Route.from(start).lineTo(score).build();
        Route toPickup = Route.from(score)
                .curveTo(pickup, Vec.of(70, 60)).maxPower(0.7)
                .at(0.6, () -> telemetry.addLine("intake on"))   // fires once at 60% of the segment
                .build();
        Route back = Route.from(pickup).lineTo(score).build();

        // Replace the instant(...) bodies with your own subsystem commands.
        Command auto = sequential(
                follow(flow, toScore),
                instant(() -> { /* shoot */ }),
                waitMs(300),
                follow(flow, toPickup),
                parallel(follow(flow, back), instant(() -> { /* spin up flywheel */ })),
                instant(() -> { /* shoot */ })
        );

        waitForStart();
        schedule(auto);

        while (opModeIsActive()) {
            Scheduler.execute();
            flow.update();
            telemetry.addData("pose", flow.pose());
            telemetry.update();
        }
    }
}
