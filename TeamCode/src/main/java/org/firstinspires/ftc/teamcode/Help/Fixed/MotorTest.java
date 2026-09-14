package org.firstinspires.ftc.teamcode.Help.Fixed;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp (name = "DcMotor", group = "test")
@Disabled
public class MotorTest extends OpMode {
    DcMotor motor;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.1f){
            motor.setPower(1.0);
        }
        if (gamepad1.left_trigger > 0.1f){
            motor.setPower(-1.0);
        }
        else {
            motor.setPower(0);
        }
    }
}