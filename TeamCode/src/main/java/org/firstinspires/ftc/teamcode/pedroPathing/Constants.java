package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
//NOTE: Automatic Tunners
            .forwardZeroPowerAcceleration(-39.0565) //fixme: -47.334483
            .lateralZeroPowerAcceleration(-84.06998750565103) //fixme: -89.748467
//NOTE: Translational PID
            .translationalPIDFCoefficients(new PIDFCoefficients(0.075, 0, 0.0009, 0.03)) //fixme: PIDFCoefficients(0.045, 0, 0.003, 0.06)
//            .translationalPIDFSwitch(4)
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.005, 0.0006));
//NOTE: Heading PID
            .headingPIDFCoefficients(new PIDFCoefficients(0.63, 0, 0.0025, 0.03)) //fixme: PIDFCoefficients(0.9, 0, 0.03, 0.05)
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.07, 0.01))
//NOTE: Drive PID
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.07, 0, 0.003, 0, 0.03))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.05, 0.6, 0.01))
            .drivePIDFSwitch(15)

//NOTE: Centripental PID
            .centripetalScaling(0.00000005); //fixme: 0.00091

    //public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(69.601863)
            .yVelocity(38.817045);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(133.1)
            .strafePodX(-43.7)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.185,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
