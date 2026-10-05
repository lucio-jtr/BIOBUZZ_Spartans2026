package org.firstinspires.ftc.teamcode.Autonomous.Tests.BLUE_AUTONOMOUS;


import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "BLUE_AUTO_DOWN_REV3")
public class BLUE_AUTO_DOWN_REV3 extends OpMode {

    private Follower follower;

    // =====================================================
    // POSE FACTORY
    // =====================================================

    private Pose

    // =====================================================
    // POSES
    // =====================================================




    // =====================================================
    // PATHS
    // =====================================================




    // =====================================================
    // INIT
    // =====================================================

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);



        follower.update();

        telemetry.addLine("Autonomous listo");
        telemetry.update();
    }


    // =====================================================
    // START
    // =====================================================

    @Override
    public void start() {



    }


    // =====================================================
    // LOOP
    // =====================================================

    @Override
    public void loop() {

        follower.update();








        telemetry.update();
    }
}