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
import org.firstinspires.ftc.teamcode.pedroPathing.OpMaster.Mecanismos;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@Autonomous(group = "A")
public class BLUE_AUTO_DOWN_REV1 extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private double time_Stamp;

    private final Pose startingPose = new Pose(55.000, 9.000, Math.toRadians(90));            //TL:Path #1
    private final Pose search_pose = new Pose(61.000, 12.000, Math.toRadians(90));            //TL:Path #1

    private final Pose shoot_Pose = new Pose(60.000, 12.000, Math.toRadians(120));            //TL:Path #2 TODO: SHOOT

    private final Pose fst_itk_pose = new Pose(48.000, 32.000, Math.toRadians(180));          //TL:Path #3

    private final Pose fst_itk_1 = new Pose(35.000, 32.000, Math.toRadians(180));             //TL:Path #4

    private final Pose fst_itk_2 = new Pose(30.000, 32.000, Math.toRadians(180));             //TL:Path #5

    private final Pose fst_itk_3 = new Pose(8.000, 32.000, Math.toRadians(180));             //TL:Path #6

    private final Pose shoot_Pose_2 = new Pose(60.000, 12.000, Math.toRadians(120));          //TL:Path #7 TODO: SECOND SHOOT

    private final Pose parking_emergency_pose = new Pose(40.000, 9.000, Math.toRadians(180));  //TL:Path #8 fixme: EMERGENCY


    private Path start_path;
    private PathChain snd_path, trd_path, fth_path, fvth_path, sxth_path, svnth_path, egth_path, nnth_path, tenth_path, elvnth_path,final_a, final_b, twlfth_path, thirtnth_path, frtnth_path, fftnth_path, sxtnth_path, svntnth_path, eightnth_path, nntnth_path, prk_em_path;

    public void buildPaths() {

        start_path = new Path(new BezierLine(startingPose, search_pose));
        start_path.setLinearHeadingInterpolation(startingPose.getHeading(), search_pose.getHeading());

        snd_path = follower.pathBuilder()
                .addPath(new BezierLine(search_pose, shoot_Pose))
                .setLinearHeadingInterpolation(search_pose.getHeading(), shoot_Pose.getHeading())
                .build();

        trd_path = follower.pathBuilder()
                .addPath(new BezierLine(shoot_Pose, fst_itk_pose))
                .setLinearHeadingInterpolation(shoot_Pose.getHeading(), fst_itk_pose.getHeading())
                .build();

        fth_path = follower.pathBuilder()
                .addPath(new BezierLine(fst_itk_pose, fst_itk_1))
                .setLinearHeadingInterpolation(fst_itk_pose.getHeading(), fst_itk_1.getHeading())
                .build();

        fvth_path = follower.pathBuilder()
                .addPath(new BezierLine(fst_itk_1, fst_itk_2))
                .setLinearHeadingInterpolation(fst_itk_1.getHeading(), fst_itk_2.getHeading())
                .build();

        sxth_path = follower.pathBuilder()
                .addPath(new BezierLine(fst_itk_2, fst_itk_3))
                .setLinearHeadingInterpolation(fst_itk_2.getHeading(), fst_itk_3.getHeading())
                .build();

        svnth_path = follower.pathBuilder()
                .addPath(new BezierLine(fst_itk_3, shoot_Pose_2))
                .setLinearHeadingInterpolation(fst_itk_3.getHeading(), shoot_Pose_2.getHeading())
                .build();

        prk_em_path = follower.pathBuilder()
                .addPath(new BezierLine(shoot_Pose_2, parking_emergency_pose))
                .setLinearHeadingInterpolation(shoot_Pose_2.getHeading(), parking_emergency_pose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        double actual_time = pathTimer.getElapsedTimeSeconds();
        telemetry.addData("actualTime: ", actual_time);

        switch (pathState) {
            case 0: //start to obelisk
                follower.setMaxPower(0.55);
                follower.followPath(start_path);
                time_Stamp = actual_time;
                setPathState(1);
                break;
            case 1://obelisk to shoot
                if (!follower.isBusy() && actual_time >= time_Stamp + 3) {
                    follower.followPath(snd_path,true);
                    setPathState(2);
                    follower.setMaxPower(7);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    mecanism.shootNear();                                                               //TODO: SHOOT
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && !mecanism.isShooting) {
                    follower.followPath(trd_path, true);
                    setPathState(5);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    time_Stamp = actual_time;
                    setPathState(5);
                }
                break;
            case 5:
                if ((!follower.isBusy()) && (actual_time >= time_Stamp + 0.1)){
                    follower.setMaxPower(0.4);
                    mecanism.intake(-0.8);
                    follower.followPath(fth_path, true);                                    //TL:FIRST INTAKE, 1
                    mecanism.intakerOFF();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    time_Stamp = actual_time;
                    setPathState(7);
                }
                break;
            case 7:
                if ((actual_time >= time_Stamp + 0.0) && (actual_time < time_Stamp + 0.5)) {
                    mecanism.intakerON();
                }
                if (actual_time >= time_Stamp + 0.6) {
                    mecanism.A = 2;
                    mecanism.B = 0;
                    mecanism.C = 0;
                    mecanism.barril.setPosition(Mecanismos.Bin);
                    mecanism.actualPos = 'b';
                }
                if (actual_time >= time_Stamp + 0.9){
                    follower.followPath(fvth_path, true);                                   //TL:FIRST INTAKE, 2
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    time_Stamp = actual_time;
                    setPathState(9);
                }
                break;
            case 9:
                if ((actual_time >= time_Stamp + 0.0) && (actual_time < time_Stamp + 0.5)) {
                    mecanism.intakerON();
                }
                if (actual_time >= time_Stamp + 0.6) {
                    mecanism.B = 1;
                    mecanism.C = 0;
                    mecanism.barril.setPosition(Mecanismos.Cin);
                    mecanism.actualPos = 'c';
                }
                if (actual_time >= time_Stamp + 0.5){
                    mecanism.intakerOFF();
                }
                if (actual_time >= time_Stamp + 0.9){
                    follower.followPath(sxth_path, true);                                   //TL:FIRST INTAKE, 3
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    time_Stamp = actual_time;
                    setPathState(11);
                }
            case 11:
                if ((actual_time >= time_Stamp + 0.2) && (actual_time < time_Stamp + 0.7)) {
                    mecanism.intakerON();
                    if (actual_time >= time_Stamp + 0.7) {
                        mecanism.C = 1;
                    }
                    follower.setMaxPower(0.55);
                    mecanism.intake(0);
                    follower.followPath(svnth_path, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    mecanism.shootNear();                                                               //TODO:SECOND SHOOT
                    setPathState(13);
                }
                break;
            case 13:
                if ((!follower.isBusy()) && (!mecanism.isShooting)) {
                    follower.followPath(prk_em_path, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    setPathState(-1);
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
        //TL: APRIL TAG DETECTION

        List<AprilTagDetection> currentDetections = mecanism.aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((detection.id == 21)) {
                    mecanism.DESIRED_TAG_ID = 21;
                    break;
                }if ((detection.id == 22)) {
                    mecanism.DESIRED_TAG_ID = 22;
                    break;
                }if ((detection.id == 23)) {
                    mecanism.DESIRED_TAG_ID = 23;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

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