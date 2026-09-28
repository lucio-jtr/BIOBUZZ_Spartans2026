package org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Tests;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "TeleOp_Auto_Heading_REV3")
public class TeleOp_REV3 extends OpMode {

    private Follower follower;
    private TelemetryManager telemetryM;


    // =====================================================
    // HEADING HOLD
    // =====================================================

    // Ángulo que actualmente queremos mantener
    private double targetHeading;

    // Indica que el driver acaba de terminar de girar
    private boolean wasTurning = false;

    // Indica que estamos esperando a que el robot se estabilice
    private boolean capturingHeading = false;

    // Momento en el que el driver soltó el joystick
    private long headingReleaseTime;


    // =====================================================
    // PID
    // =====================================================

    private double headingIntegral = 0;
    private double previousHeadingError = 0;
    private long previousTime;


    // =====================================================
    // AJUSTES DESDE PANELS
    // =====================================================

    public static double KP = 0.79;
    public static double KI = 0.0;
    public static double KD = 0.0025;

    // Zona muerta del joystick derecho
    public static double TURN_DEADBAND = 0.05;

    // Tiempo que esperamos después de soltar
    // el joystick antes de capturar el nuevo heading.
    //
    // 0.15 = 150 milisegundos
    public static double HEADING_CAPTURE_DELAY = 0.20;


    @Override
    public void init() {

        // Crear Follower
        follower = Constants.createFollower(hardwareMap);

        // Posición inicial
        follower.setStartingPose(new Pose(72, 72));

        // Telemetría de Panels
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Actualizar localización
        follower.update();

        // El heading inicial será nuestro primer objetivo
        targetHeading = follower.getPose().getHeading();

        previousTime = System.nanoTime();

        telemetry.addLine("TeleOp listo");
        telemetry.addData(
                "Starting Heading",
                Math.toDegrees(targetHeading)
        );
        telemetry.update();
    }


    @Override
    public void start() {

        // Activar control TeleOp de Pedro
        follower.startTeleopDrive();

        // El heading donde comienza el partido
        // será el primer heading que debemos mantener
        targetHeading = follower.getPose().getHeading();

        // Reiniciar PID
        headingIntegral = 0;
        previousHeadingError = 0;

        previousTime = System.nanoTime();

        wasTurning = false;
        capturingHeading = false;
    }


    @Override
    public void loop() {

        // =====================================================
        // ACTUALIZAR LOCALIZACIÓN
        // =====================================================

        follower.update();


        // =====================================================
        // CONTROLES DEL DRIVER
        // =====================================================

        double forward = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double driverTurn = -gamepad1.right_stick_x;


        // =====================================================
        // HEADING ACTUAL
        // =====================================================

        double currentHeading = follower.getPose().getHeading();


        // =====================================================
        // ¿EL DRIVER ESTÁ GIRANDO?
        // =====================================================

        boolean driverTurning =
                Math.abs(driverTurn) > TURN_DEADBAND;


        double headingCorrection = 0;


        // =====================================================
        // 1. DRIVER ESTÁ GIRANDO
        // =====================================================

        if (driverTurning) {

            /*
             * EL DRIVER TIENE CONTROL TOTAL.
             *
             * El Heading Hold queda completamente desactivado.
             */

            headingCorrection = 0;

            wasTurning = true;
            capturingHeading = false;


            // Reiniciar PID
            headingIntegral = 0;
            previousHeadingError = 0;

            previousTime = System.nanoTime();
        }


        // =====================================================
        // 2. DRIVER ACABA DE SOLTAR
        // =====================================================

        else if (wasTurning && !capturingHeading) {

            /*
             * El driver acaba de soltar el joystick.
             *
             * NO capturamos el heading inmediatamente.
             *
             * Primero dejamos que el robot termine de
             * girar por su propia inercia.
             */

            capturingHeading = true;

            headingReleaseTime = System.nanoTime();

            // PID todavía apagado
            headingCorrection = 0;

            headingIntegral = 0;
            previousHeadingError = 0;

            previousTime = System.nanoTime();
        }


        // =====================================================
        // 3. ESPERANDO A QUE EL ROBOT SE ESTABILICE
        // =====================================================

        else if (capturingHeading) {

            long currentTime = System.nanoTime();

            double elapsedTime =
                    (currentTime - headingReleaseTime)
                            / 1_000_000_000.0;


            /*
             * Durante este pequeño período:
             *
             * - No corregimos
             * - No intentamos regresar
             * - Dejamos que el robot termine de girar
             */

            headingCorrection = 0;


            // =================================================
            // YA PASÓ EL TIEMPO DE ESTABILIZACIÓN
            // =================================================

            if (elapsedTime >= HEADING_CAPTURE_DELAY) {

                /*
                 * ESTE ES EL MOMENTO IMPORTANTE.
                 *
                 * Volvemos a leer el heading ACTUAL.
                 *
                 * Ese será el nuevo objetivo.
                 *
                 * Ejemplo:
                 *
                 * 90° → driver gira → 180°
                 *             ↓
                 *          suelta
                 *             ↓
                 * robot termina de estabilizarse
                 *             ↓
                 * currentHeading = 181°
                 *             ↓
                 * targetHeading = 181°
                 */

                currentHeading =
                        follower.getPose().getHeading();

                targetHeading = currentHeading;


                // Reiniciar PID
                headingIntegral = 0;
                previousHeadingError = 0;

                previousTime = System.nanoTime();


                // Ya capturamos el nuevo heading
                capturingHeading = false;
                wasTurning = false;
            }
        }


        // =====================================================
        // 4. HEADING HOLD NORMAL
        // =====================================================

        else {

            /*
             * Aquí el driver NO está girando
             * y ya tenemos un target establecido.
             */

            double headingError =
                    normalizeAngle(
                            targetHeading - currentHeading
                    );


            // Tiempo
            long currentTime = System.nanoTime();

            double deltaTime =
                    (currentTime - previousTime)
                            / 1_000_000_000.0;

            previousTime = currentTime;


            if (deltaTime <= 0) {
                deltaTime = 0.001;
            }


            // =================================================
            // P
            // =================================================

            double proportional =
                    KP * headingError;


            // =================================================
            // I
            // =================================================

            headingIntegral +=
                    headingError * deltaTime;

            double integral =
                    KI * headingIntegral;


            // =================================================
            // D
            // =================================================

            double derivative =
                    (headingError - previousHeadingError)
                            / deltaTime;

            double derivativeTerm =
                    KD * derivative;


            previousHeadingError =
                    headingError;


            // =================================================
            // PID
            // =================================================

            headingCorrection =
                    proportional
                            + integral
                            + derivativeTerm;


            // Limitar corrección
            headingCorrection =
                    Math.max(
                            -1.0,
                            Math.min(
                                    1.0,
                                    headingCorrection
                            )
                    );
        }


        // =====================================================
        // GIRO FINAL
        // =====================================================

        double finalTurn;


        if (driverTurning) {

            /*
             * Driver manda directamente.
             */
            finalTurn = driverTurn;

        } else if (capturingHeading) {

            /*
             * Estamos esperando que el robot termine
             * de asentarse.
             *
             * NO corregimos.
             */
            finalTurn = 0;

        } else {

            /*
             * Heading Hold está activo.
             */
            finalTurn = headingCorrection;
        }


        // =====================================================
        // DRIVE
        // =====================================================

        follower.setTeleOpDrive(
                forward,
                lateral,
                finalTurn,
                true
        );


        // =====================================================
        // PANELS
        // =====================================================

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
                !driverTurning && !capturingHeading
        );

        telemetryM.addData(
                "Capturing Heading",
                capturingHeading
        );

        telemetryM.addData(
                "Capture Delay",
                HEADING_CAPTURE_DELAY
        );

        telemetryM.update(telemetry);


        // =====================================================
        // TELEMETRÍA DRIVER STATION
        // =====================================================

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

        telemetry.addData(
                "Heading Hold",
                !driverTurning && !capturingHeading
        );

        telemetry.update();
    }


    // =====================================================
    // NORMALIZAR ÁNGULO
    // =====================================================

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