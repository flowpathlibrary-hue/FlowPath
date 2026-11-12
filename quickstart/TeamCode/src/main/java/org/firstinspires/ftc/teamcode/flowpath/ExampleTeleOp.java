package org.firstinspires.ftc.teamcode.flowpath;

import com.flowpath.Flow;
import com.flowpath.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Flow TeleOp (field centric)", group = "FlowPath")
public class ExampleTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Flow flow = Robot.create(hardwareMap);
        flow.setPose(Pose.of(0, 0, 0));
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.options) flow.setPose(new Pose(flow.pose().x, flow.pose().y, 0)); // re-zero heading
            flow.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            flow.update();
        }
    }
}
