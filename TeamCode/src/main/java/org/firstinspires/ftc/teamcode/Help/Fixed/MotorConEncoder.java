package org.firstinspires.ftc.teamcode.Help.Fixed;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "test", name = "RPM Motor Control")
public class MotorConEncoder extends LinearOpMode {

    DcMotorEx motor;
    DcMotorEx motor2;
    DcMotorEx cannon;

    static final double TICKS_PER_REV = 537.7;
    double targetRPM = 0;
    double power = 0;

    @Override
    public void runOpMode() {


        motor = hardwareMap.get(DcMotorEx.class, "CannonR");
        motor2 = hardwareMap.get(DcMotorEx.class, "CannonL");
        cannon = hardwareMap.get(DcMotorEx.class, "Cannon");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        cannon.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) power = 100;
            if (gamepad1.dpad_up) power = 50;
            if (gamepad1.y) power = 0;

            if (gamepad1.bWasReleased() && power <= 100){power ++;
                telemetry.speak("nigger");}
            if (gamepad1.xWasReleased() && power >= 0){power --;
                telemetry.speak("stupid");}

            targetRPM = (power * 312) / 100;

            setMotorRPM(targetRPM);

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Power: ", power + "%");
            telemetry.addData("Current RPM", getMotorRPM());

            telemetry.update();
        }
    }

    void setMotorRPM(double rpm) {
        double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
        motor.setVelocity(ticksPerSecond);
        motor2.setVelocity(ticksPerSecond);
        cannon.setPower(power);
    }

    double getMotorRPM() {
        return (motor.getVelocity() * 60.0) / TICKS_PER_REV;
    }
}