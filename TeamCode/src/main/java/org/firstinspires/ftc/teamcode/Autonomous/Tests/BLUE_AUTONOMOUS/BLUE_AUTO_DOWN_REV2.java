package org.firstinspires.ftc.teamcode.Autonomous.Tests.BLUE_AUTONOMOUS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;

//import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BLUE_AUTO_DOWN_REV2 extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    //TL:------------------------- POSES ---------------------------
    //p.of(x, y, heading);
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose park = p.of(48, 48, 90);

    /*private final Pose startPose = poseFactory.of(24, 24, 0);
    private final Pose scorePose = poseFactory.of(48, 48, 90);
    private final Pose parkPose = poseFactory.of(72, 48, 90);*/


    //TL:------------------------- CURVE ---------------------------
    private final Pose controlPose = p.of(36, 60, 45);

    //TL:--------------------- CREATE PATHS ------------------------
    private Path startToScore() {
        return line(startPose, park).linear(startPose, park);
    }
    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }

    //TL:---------------------- CURVE PATHS ------------------------
    /*private Path park() {
        return curve(startPose, controlPose, park).linear(startPose, park);
    }*/

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                // Add mechanism commands here.
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        //follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }
    @Override
    public void start() {
        schedule(follow(follower, park()));
        // using the same park path from the last page
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        // add your other methods needed in the loop here
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}