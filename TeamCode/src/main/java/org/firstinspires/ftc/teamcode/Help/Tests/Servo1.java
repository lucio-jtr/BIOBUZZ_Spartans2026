package org.firstinspires.ftc.teamcode.Help.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Servo1 extends OpMode{
    Servo servomotor;
    double velocity = 0;
    boolean prevrightbumper = false;
    boolean prevleftbumper = false;

    @Override
    public void init(){
        servomotor = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop(){
        if (gamepad1.right_bumper && !prevrightbumper){
            velocity += 0.003;
            if (velocity > 1) velocity = 1.0;
        }
        if (gamepad1.left_bumper && !prevleftbumper){
            velocity -= 0.003;
        }
        if (gamepad1.x){
            servomotor.setPosition(0.0);
        }
        prevrightbumper = gamepad1.right_bumper;
        prevleftbumper = gamepad1.left_bumper;

        servomotor.setPosition(velocity);

        telemetry.addData("Velocidad constante actualizada", velocity);
        telemetry.update();
    }
}
