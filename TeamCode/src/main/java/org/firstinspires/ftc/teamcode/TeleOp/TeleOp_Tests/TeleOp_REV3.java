package org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Mecanism_Tests.Mecanism_REV1;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "TeleOp_Auto_Heading_REV3")
public class TeleOp_REV3 extends OpMode {
    Mecanism_REV1 mecanism = new Mecanism_REV1();

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
    // el joystick antes de capturar el nuevo heading
    public static double HEADING_CAPTURE_DELAY = 0.20;


    // =====================================================
    // MODO LENTO
    // =====================================================

    // Porcentaje de potencia cuando se mantiene LT
    public static double SLOW_MODE = 0.30;


    @Override
    public void init() {
        mecanism.initAll(hardwareMap);
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

            // El driver tiene control absoluto del giro
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

            // Comenzamos a esperar a que el robot
            // termine de girar por inercia
            capturingHeading = true;

            headingReleaseTime = System.nanoTime();

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


            // Todavía no capturamos el nuevo heading
            headingCorrection = 0;


            if (elapsedTime >= HEADING_CAPTURE_DELAY) {

                // Ahora sí tomamos el heading REAL
                // después de que terminó la inercia
                currentHeading = follower.getPose().getHeading();

                targetHeading = currentHeading;

                // Reiniciar PID
                headingIntegral = 0;
                previousHeadingError = 0;

                previousTime = System.nanoTime();

                capturingHeading = false;
                wasTurning = false;
            }
        }


        // =====================================================
        // 4. HEADING HOLD
        // =====================================================

        if (!driverTurning && !capturingHeading) {

            double headingError =
                    normalizeAngle(
                            targetHeading - currentHeading
                    );


            // Tiempo transcurrido
            long currentTime = System.nanoTime();

            double deltaTime =
                    (currentTime - previousTime)
                            / 1_000_000_000.0;

            previousTime = currentTime;


            if (deltaTime <= 0) {
                deltaTime = 0.001;
            }


            // -------------------------------------------------
            // P
            // -------------------------------------------------

            double proportional =
                    KP * headingError;


            // -------------------------------------------------
            // I
            // -------------------------------------------------

            headingIntegral +=
                    headingError * deltaTime;

            double integral =
                    KI * headingIntegral;


            // -------------------------------------------------
            // D
            // -------------------------------------------------

            double derivative =
                    (headingError - previousHeadingError)
                            / deltaTime;

            double derivativeTerm =
                    KD * derivative;


            previousHeadingError =
                    headingError;


            // -------------------------------------------------
            // PID FINAL
            // -------------------------------------------------

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

            // El driver está girando manualmente
            finalTurn = driverTurn;

        } else if (capturingHeading) {

            // Mientras esperamos estabilización,
            // no hacemos ninguna corrección
            finalTurn = 0;

        } else {

            // Heading Hold
            finalTurn = headingCorrection;
        }


        // =====================================================
        // MODO LENTO
        // =====================================================

        /*
         * Si LT está presionado:
         *
         *     forward  → 30%
         *     lateral  → 30%
         *     giro     → 30%
         *
         * Esto funciona tanto para el giro manual
         * como para la corrección de heading.
         */

        if (gamepad1.left_trigger > 0) {

            forward *= SLOW_MODE;
            lateral *= SLOW_MODE;
            finalTurn *= SLOW_MODE;
        }


        // =====================================================
        // MOVER ROBOT
        // =====================================================

        follower.setTeleOpDrive(
                forward,
                lateral,
                finalTurn,
                true
        );


        // =====================================================
        // AQUÍ VAN LOS MECANISMOS
        // =====================================================
        if (gamepad1.right_trigger > 0.1) {
            mecanism.intake(-0.8);
        }
        else {
            mecanism.intake(0);
        }

        // =====================================================
        // TELEMETRÍA
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
                !driverTurning
        );

        telemetryM.addData(
                "Slow Mode",
                gamepad1.left_trigger > 0
        );

        telemetryM.update(telemetry);


        // =====================================================
        // TELEMETRÍA NORMAL
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