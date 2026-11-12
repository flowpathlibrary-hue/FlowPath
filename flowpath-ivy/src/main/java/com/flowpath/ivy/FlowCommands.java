package com.flowpath.ivy;

import com.flowpath.Flow;
import com.flowpath.Pose;
import com.flowpath.Route;
import com.pedropathing.ivy.Command;

/**
 * Ivy commands for FlowPath. Each requires the Flow instance, so two movement commands can never
 * fight over the drivetrain (Ivy cancels or queues them according to its priority rules).
 *
 * Loop order in your OpMode:  Scheduler.execute();  flow.update();
 */
public final class FlowCommands {
    private FlowCommands() {}

    /** Run a route; finishes when the robot has settled at the end (or is close, if the route has holdEnd(false)). */
    public static Command follow(Flow flow, Route route) {
        return Command.build()
                .setStart(() -> flow.run(route))
                .setDone(() -> !flow.busy())
                .setEnd(ec -> { if (flow.busy()) flow.stop(); })
                .requiring(flow);
    }

    /** Move to a pose and hold it until settled. */
    public static Command hold(Flow flow, Pose target) {
        return Command.build()
                .setStart(() -> flow.hold(target))
                .setDone(() -> !flow.busy())
                .setEnd(ec -> { })
                .requiring(flow);
    }

    /** Straight line from wherever the robot is when the command starts to the target pose. */
    public static Command driveTo(Flow flow, Pose target) {
        return Command.build()
                .setStart(() -> flow.run(Route.from(flow.pose()).lineTo(target).build()))
                .setDone(() -> !flow.busy())
                .setEnd(ec -> { if (flow.busy()) flow.stop(); })
                .requiring(flow);
    }

    /** Rotate in place to an absolute heading in degrees. */
    public static Command turnTo(Flow flow, double headingDeg) {
        return Command.build()
                .setStart(() -> flow.turnTo(headingDeg))
                .setDone(() -> !flow.busy())
                .setEnd(ec -> { })
                .requiring(flow);
    }

    public static Command stop(Flow flow) {
        return Command.build()
                .setStart(flow::stop)
                .setDone(() -> true)
                .setEnd(ec -> { })
                .requiring(flow);
    }
}
