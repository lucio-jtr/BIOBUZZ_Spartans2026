package org.firstinspires.ftc.teamcode.TeleOp.Mecanism_Tests;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.pedroPathing.Tests.TestColorSensorMecanism;
import org.firstinspires.ftc.teamcode.DECODE.Tests.TestColorSensorMecanism;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**TL: PORTS
 *>>Control Hub:
 * motores:
 *  ("rightFront")  //0
 *  ("rightRear")    //2
 *  ("leftRear")  //3
 *  ("leftFront")    //1
 * Servos:
 *
 * I2C:
 *  pinpoint = 2

 *>>Expansion Hub;
 * motores:
 *  Intake = 0
 * servos:
 *
 */

public class Mecanism_REV1 {
    // Tl:========= INTAKE =========
    public DcMotor intake;

    //TL: ============= INIT =================
    public void initAll(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class, "Intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //TL: ============== INTAKE ==============
    public void intake(double pow){
        intake.setPower(pow);
    }
}