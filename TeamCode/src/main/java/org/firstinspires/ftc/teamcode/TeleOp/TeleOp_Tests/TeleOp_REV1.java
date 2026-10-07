package org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Disabled
@TeleOp(name = "TeleOp_Auto_Heading")
public class TeleOp_REV1 extends OpMode {

    // =========================
    // PEDRO PATHING
    // =========================

    private Follower follower;

    // =========================
    // PANELS
    // =========================

    private TelemetryManager telemetryM;

    // =========================
    // HEADING HOLD
    // =========================

    // Heading que queremos mantener
    private double targetHeading;

    // Para saber si el driver estaba girando
    private boolean wasTurning = false;

    // PID
    private double headingIntegral = 0;
    private double previousHeadingError = 0;

    private long previousTime;

    // =========================
    // PID VALUES
    // =========================

    public static double KP = 0.79;
    public static double KI = 0.0;
    public static double KD = 0.0025;

    // Deadband del joystick de giro
    public static double TURN_DEADBAND = 0.05;

    // =========================
    // TELEOP INIT
    // =========================

    @Override
    public void init() {

        // Crear Follower
        follower = Constants.create(hardwareMap);

        // Pose inicial
        //follower.setPose(startPose);

        // Inicializar Panels
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Actualizar Follower
        follower.update();

        // El heading inicial será nuestro primer objetivo
        //targetHeading = follower.getPose().getHeading();

        previousTime = System.nanoTime();

        telemetry.addLine("TeleOp listo");
        telemetry.addData(
                "Starting Heading",
                Math.toDegrees(targetHeading)
        );

        telemetry.update();
    }

    // =========================
    // START
    // =========================

    @Override
    public void start() {
        // Actualizar tiempo del PID
        previousTime = System.nanoTime();

        // Reiniciar PID
        headingIntegral = 0;
        previousHeadingError = 0;

        // El heading actual es nuestro objetivo inicial
        targetHeading = follower.pose().heading();

        wasTurning = false;
    }

    // =========================
    // LOOP
    // =========================

    @Override
    public void loop() {

        // Actualizar localización
        follower.update();

        // =========================
        // JOYSTICKS
        // =========================

        double forward = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double driverTurn = -gamepad1.right_stick_x;

        // =========================
        // DETECTAR GIRO DEL DRIVER
        // =========================

        boolean driverTurning =
                Math.abs(driverTurn) > TURN_DEADBAND;

        // =========================
        // HEADING ACTUAL
        // =========================

        double currentHeading = follower.pose().heading();

        // =========================
        // HEADING HOLD
        // =========================

        double headingCorrection = 0;

        if (driverTurning) {

            // ---------------------------------
            // EL DRIVER ESTÁ GIRANDO
            // ---------------------------------

            // Dejamos que el driver controle
            // completamente el giro.

            headingCorrection = 0;

            // Reiniciamos el PID
            headingIntegral = 0;

            previousHeadingError = 0;

            wasTurning = true;

        } else {

            // ---------------------------------
            // EL DRIVER NO ESTÁ GIRANDO
            // ---------------------------------

            // Si acaba de soltar el joystick,
            // guardamos el heading actual como
            // nuevo objetivo.

            if (wasTurning) {

                targetHeading = currentHeading;

                // Reiniciar PID
                headingIntegral = 0;
                previousHeadingError = 0;

                wasTurning = false;
            }

            // =========================
            // CALCULAR ERROR
            // =========================

            double headingError =
                    normalizeAngle(
                            targetHeading - currentHeading
                    );

            // =========================
            // TIEMPO
            // =========================

            long currentTime = System.nanoTime();

            double deltaTime =
                    (currentTime - previousTime) / 1_000_000_000.0;

            previousTime = currentTime;

            // Evitar problemas si dt es demasiado pequeño
            if (deltaTime <= 0) {
                deltaTime = 0.001;
            }

            // =========================
            // P - PROPORCIONAL
            // =========================

            double proportional =
                    KP * headingError;

            // =========================
            // I - INTEGRAL
            // =========================

            headingIntegral +=
                    headingError * deltaTime;

            double integral =
                    KI * headingIntegral;

            // =========================
            // D - DERIVATIVO
            // =========================

            double derivative =
                    (headingError - previousHeadingError)
                            / deltaTime;

            double derivativeTerm =
                    KD * derivative;

            previousHeadingError = headingError;

            // =========================
            // PID FINAL
            // =========================

            headingCorrection =
                    proportional
                            + integral
                            + derivativeTerm;

            // Limitar corrección
            headingCorrection =
                    Math.max(-1.0,
                            Math.min(1.0, headingCorrection));
        }

        // =========================
        // MOVIMIENTO
        // =========================

        double finalTurn;

        if (driverTurning) {

            // El driver tiene el control
            finalTurn = driverTurn;

        } else {

            // Heading Hold tiene el control
            finalTurn = headingCorrection;
        }

        // =========================
        // PEDRO PATHING
        // =========================

        follower.manual(
                forward,
                lateral,
                finalTurn
        );

        // =========================
        // PANELS
        // =========================

        telemetryM.debug(
                "Heading Hold activo cuando el driver no gira."
        );

        telemetryM.addData(
                "Current Heading",
                Math.toDegrees(currentHeading)
        );

        telemetryM.addData(
                "Target Heading",
                Math.toDegrees(targetHeading)
        );

        telemetryM.addData(
                "Heading Error",
                Math.toDegrees(
                        normalizeAngle(
                                targetHeading - currentHeading
                        )
                )
        );

        telemetryM.addData(
                "Driver Turn",
                driverTurn
        );

        telemetryM.addData(
                "Heading Correction",
                headingCorrection
        );

        telemetryM.addData(
                "Heading Hold",
                !driverTurning
        );

        telemetryM.update(telemetry);

        // =========================
        // DRIVER STATION
        // =========================

        telemetry.addData(
                "Heading",
                Math.toDegrees(currentHeading)
        );

        telemetry.addData(
                "Target",
                Math.toDegrees(targetHeading)
        );

        telemetry.addData(
                "Error",
                Math.toDegrees(
                        normalizeAngle(
                                targetHeading - currentHeading
                        )
                )
        );

        telemetry.update();
    }

    // =========================
    // NORMALIZAR ÁNGULO
    // =========================

    private double normalizeAngle(double angle) {

        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        return angle;
    }
}

/*if (!automatedDrive) {//  TL: DRIVE {GPAD_1}
            if (gamepad1.left_trigger == 0.0) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 0.3,
                    -gamepad1.left_stick_x * 0.3,
                    -gamepad1.right_stick_x * 0.3,
                    true // Robot Centric
            );
        }*/