package org.firstinspires.ftc.teamcode.flowpath;

import com.flowpath.Flow;
import com.flowpath.Pose;
import com.flowpath.Route;
import com.flowpath.Vec;
import com.flowpath.dash.Dash;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Step 2 of tuning. Open the dashboard, run this, pick a test with the gamepad and drag the sliders.
 *
 *   A = HOLD   shove the robot by hand, it should snap back without oscillating  (Translation, Heading)
 *   B = LINE   48 in forward and back                                           (Drive, Translation)
 *   X = CURVE  S-curve forward and back                                         (Curves.centripetalGain)
 *   Y = TURN   spin 0 <-> 90 deg in place                                       (Heading)
 *   BACK = stop.   Then press "Save to robot" on the dashboard.
 */
@TeleOp(name = "Flow Tuner", group = "FlowPath")
public class FlowTuner extends LinearOpMode {
    private enum Test { NONE, HOLD, LINE, CURVE, TURN }

    @Override
    public void runOpMode() {
        Flow flow = Robot.create(hardwareMap);
        Pose a = Pose.of(24, 72, 0), b = Pose.of(72, 72, 0);
        flow.setPose(a);

        Route lineOut = Route.from(a).lineTo(b).holdHeading(0).build();
        Route lineBack = Route.from(b).lineTo(a).holdHeading(0).build();
        Route curveOut = Route.from(a).curveTo(b, Vec.of(40, 110), Vec.of(56, 34)).holdHeading(0).build();
        Route curveBack = Route.from(b).curveTo(a, Vec.of(56, 34), Vec.of(40, 110)).holdHeading(0).build();

        Test test = Test.NONE;
        boolean atB = false;
        ElapsedTime pause = new ElapsedTime();

        telemetry.addLine("Open http://192.168.43.1:8099 then press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            Test pick = gamepad1.a ? Test.HOLD : gamepad1.b ? Test.LINE : gamepad1.x ? Test.CURVE : gamepad1.y ? Test.TURN : null;
            if (gamepad1.back) { test = Test.NONE; flow.stop(); }
            if (pick != null && pick != test) {
                test = pick;
                flow.setPose(a);
                atB = false;
                pause.reset();
                if (test == Test.HOLD) flow.hold(a);
                if (test == Test.TURN) flow.turnTo(90);
            }

            if (test == Test.LINE || test == Test.CURVE) {
                if (flow.busy()) pause.reset();
                else if (pause.seconds() > 0.6) {
                    boolean line = test == Test.LINE;
                    flow.run(atB ? (line ? lineBack : curveBack) : (line ? lineOut : curveOut));
                    atB = !atB;
                }
            } else if (test == Test.TURN) {
                if (flow.busy()) pause.reset();
                else if (pause.seconds() > 0.6) { flow.turnTo(atB ? 90 : 0); atB = !atB; }
            }

            flow.update();
            Dash.put("test", test);
            telemetry.addData("test", test);
            telemetry.addLine("A=hold B=line X=curve Y=turn BACK=stop");
            telemetry.addData("pose", flow.pose());
            telemetry.update();
        }
        flow.stop();
    }
}
