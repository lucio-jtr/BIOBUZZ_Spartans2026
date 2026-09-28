package org.firstinspires.ftc.teamcode.Help.Fixed;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Chassis_April_Tag")
public class Chassis2AprilTag extends LinearOpMode
{
    Chassis2AprilTagMecanism mecanism = new Chassis2AprilTagMecanism();

    @Override public void runOpMode()
    {
        mecanism.initAprilTag(hardwareMap);
        waitForStart();

        while (opModeIsActive())
        {
            mecanism.targetFound = false;
            mecanism.desiredTag  = null;

            List<AprilTagDetection> currentDetections = mecanism.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((mecanism.DESIRED_TAG_ID < 0) || (detection.id == mecanism.DESIRED_TAG_ID)) {
                        mecanism.targetFound = true;
                        mecanism.desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (mecanism.targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", mecanism.desiredTag.id, mecanism.desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", mecanism.desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", mecanism.desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", mecanism.desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && mecanism.targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (mecanism.desiredTag.ftcPose.range - mecanism.DESIRED_DISTANCE);
                double  headingError    = mecanism.desiredTag.ftcPose.bearing;
//                double  yawError        = mecanism.desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                mecanism.drive  = Range.clip(rangeError * mecanism.SPEED_GAIN, -mecanism.MAX_AUTO_SPEED, mecanism.MAX_AUTO_SPEED);
                mecanism.turn   = Range.clip(headingError * mecanism.TURN_GAIN, -mecanism.MAX_AUTO_TURN, mecanism.MAX_AUTO_TURN) ;
//                mecanism.strafe = Range.clip(-yawError * mecanism.STRAFE_GAIN, -mecanism.MAX_AUTO_STRAFE, mecanism.MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f ", mecanism.drive, mecanism.turn);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                mecanism.drive  = -gamepad1.left_stick_y / 2;  // Reduce drive rate to 50%.
                mecanism.strafe = -gamepad1.left_stick_x / 2;  // Reduce strafe rate to 50%.
                mecanism.turn   = -gamepad1.right_stick_x / 2;  // Reduce turn rate to 33%.
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", mecanism.drive, mecanism.strafe, mecanism.turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            mecanism.moveRobot(mecanism.drive, mecanism.strafe, mecanism.turn);
            sleep(10);
        }
    }
}