package org.firstinspires.ftc.teamcode.Help.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class AprilTag_Camera_REV1 extends OpMode {
    private Servo servo_pos;
    double position = 0.5;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void init() {
        servo_pos = hardwareMap.get(Servo.class, "servo");

        servo_pos.setPosition(position);


        // Inicializar el procesador de AprilTags
        aprilTag = new AprilTagProcessor.Builder().build();

        // Crear el VisionPortal con la webcam configurada en la Driver Station
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Listo para detectar AprilTags y mostrar centro X/Y");
        telemetry.update();
    }

    @Override
    public void loop() {

        List<AprilTagDetection> detections = aprilTag.getDetections();


            for (AprilTagDetection d : detections) {

                telemetry.addData("Tag ID", d.id);

                telemetry.addData("Center X", "%.2f", d.ftcPose.x);
                telemetry.addData("Center Y", "%.2f", d.ftcPose.y);
                telemetry.addData("Center Z", "%.2f", d.ftcPose.z);

                if (d.id == 34) {

                    double errorX = d.ftcPose.x;
                    double ajuste = errorX * 0.00004;

                    if (d.ftcPose.x > 4) {
                        telemetry.addLine("DERECHA");
                        position = position + 0.0001;
                        servo_pos.setPosition(position);

                    }
                    if (d.ftcPose.x < -4) {
                        telemetry.addLine("IZQUIERDA");
                        position = position - 0.0001;
                        servo_pos.setPosition(position);

                    }
                }
            }
        servo_pos.setPosition(position);

        telemetry.addData("Position", servo_pos.getPosition());
        telemetry.update();

    }
}
