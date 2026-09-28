package org.firstinspires.ftc.teamcode.Autonomous.Tests.BLUE_AUTONOMOUS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@Autonomous(name = "BLUE_AUTO_DOWN_REV1")
public class BLUE_AUTO_DOWN_REV1 extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private double time_Stamp;

    private final Pose startingPose = new Pose(56.000, 8.000, Math.toRadians(90));            //TL:Path #1
    private final Pose search_pose = new Pose(56.000, 36.000, Math.toRadians(90));            //TL:Path #1


    private Path start_path;
    private PathChain snd_path;

    public void buildPaths() {

        start_path = new Path(new BezierLine(startingPose, search_pose));
        start_path.setLinearHeadingInterpolation(startingPose.getHeading(), search_pose.getHeading());

        snd_path = follower.pathBuilder()
                .addPath(new BezierLine(search_pose, startingPose))
                .setLinearHeadingInterpolation(search_pose.getHeading(), startingPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        double actual_time = pathTimer.getElapsedTimeSeconds();
        telemetry.addData("actualTime: ", actual_time);

        switch (pathState) {
            case 0: //start to obelisk
                follower.setMaxPower(1);
                follower.followPath(start_path);
                setPathState(1);
                break;
            case 1://obelisk to shoot
                if (!follower.isBusy()) {
                    follower.followPath(snd_path,true);
                    setPathState(0);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("pathTimer: ", pathTimer);

        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startingPose);

    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}