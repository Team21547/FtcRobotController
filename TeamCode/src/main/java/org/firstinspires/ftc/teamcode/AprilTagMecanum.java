package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "Seguimiento AprilTag Mecanum Mejorado")
public class AprilTagMecanum extends LinearOpMode {

    // Declaración de motores mecanum
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // Variables para visión
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private AprilTagDetection currentDetection = null;
    private boolean tagDetected = false;

    // Constantes de configuración
    private static final boolean USE_WEBCAM = true;
    private static final String WEBCAM_NAME = "Webcam 1";

    // Nombres de los motores - ¡VERIFICA QUE ESTOS SEAN EXACTOS!
    private static final String MOTOR_LF = "leftFront";
    private static final String MOTOR_LB = "leftBack";
    private static final String MOTOR_RF = "rightFront";
    private static final String MOTOR_RB = "rightBack";

    // Umbrales para control
    private static final double BEARING_THRESHOLD = 2.0; // grados
    private static final double STRAFE_POWER = 0.3;
    private static final double MINIMUM_RANGE = 6.0; // pulgadas - distancia mínima para detenerse
    private static final double MAX_RANGE = 24.0; // pulgadas - distancia máxima para acercarse

    // Control PID simplificado para bearing
    private static final double STRAFE_KP = 0.01; // Ganancia proporcional para strafe

    // Estados de movimiento mejorados
    private enum MovementState {
        STRAFE_LEFT,
        STRAFE_RIGHT,
        FORWARD,
        BACKWARD,
        STOP
    }

    private MovementState currentState = MovementState.STOP;

    @Override
    public void runOpMode() {
        // Inicializar hardware
        if (!initializeHardware()) {
            telemetry.addData("ERROR", "No se pudo inicializar hardware");
            telemetry.update();
            return;
        }

        // Inicializar visión
        initializeAprilTag();

        // Esperar a que el usuario presione START
        telemetry.addData("Estado", "Esperando START...");
        telemetry.addData("Instrucciones", "Presiona START para comenzar");
        telemetry.addData("Botones:", "A=Parada emergencia, B=Reset, X=Pausa cámara, Y=Reanudar");
        telemetry.update();

        waitForStart();

        // Bucle principal
        while (opModeIsActive()) {
            // Actualizar detección de AprilTag
            updateCurrentDetection();

            // Controlar los motores basado en el AprilTag
            if (!handleEmergencyControls()) {
                controlMecanumBasedOnTag();
            }

            // Mostrar telemetría
            displayTelemetry();

            // Pequeña pausa
            sleep(20);
        }

        // Limpiar recursos
        cleanup();
    }

    /**
     * Inicializa los motores
     * @return true si todo OK
     */
    private boolean initializeHardware() {
        try {
            // Intentar obtener los motores con nombres simplificados
            leftFront = hardwareMap.get(DcMotor.class, MOTOR_LF);
            leftBack = hardwareMap.get(DcMotor.class, MOTOR_LB);
            rightFront = hardwareMap.get(DcMotor.class, MOTOR_RF);
            rightBack = hardwareMap.get(DcMotor.class, MOTOR_RB);

            telemetry.addData("Hardware", "Motores encontrados!");

        } catch (Exception e) {
            telemetry.addData("Error", "No se encontraron los motores con nombres: " +
                    MOTOR_LF + ", " + MOTOR_LB + ", " + MOTOR_RF + ", " + MOTOR_RB);
            telemetry.addData("Sugerencia", "Verifica los nombres en la configuración");
            listAvailableDevices();
            return false;
        }

        // Configurar dirección de los motores para MECANUM
        // IMPORTANTE: Ajusta según la orientación de tus motores
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Configurar comportamiento BRAKE para mayor precisión
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configurar modos
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return true;
    }

    /**
     * Lista todos los dispositivos disponibles
     */
    private void listAvailableDevices() {
        telemetry.addLine("=== DISPOSITIVOS DISPONIBLES ===");

        int motorCount = 0;
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            String name = "Desconocido";
            try {
                name = hardwareMap.getNamesOf(motor).iterator().next();
            } catch (Exception e) {
                name = "Motor " + motorCount;
            }
            telemetry.addData("Motor " + motorCount, name);
            motorCount++;
        }

        telemetry.addLine("\nSUGERENCIA: Usa estos nombres en la configuración:");
        telemetry.addData("", "leftFront, leftBack, rightFront, rightBack");
    }

    /**
     * Inicializa el procesador de AprilTag
     */
    private void initializeAprilTag() {
        try {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();

            VisionPortal.Builder builder = new VisionPortal.Builder();

            if (USE_WEBCAM) {
                WebcamName webcam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
                builder.setCamera(webcam);
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            builder.setCameraResolution(new android.util.Size(640, 480));
            builder.addProcessor(aprilTagProcessor);
            visionPortal = builder.build();

            telemetry.addData("Visión", "AprilTag inicializado OK");

        } catch (Exception e) {
            telemetry.addData("ERROR Visión", e.getMessage());
        }
    }

    /**
     * Actualiza la detección actual
     */
    private void updateCurrentDetection() {
        if (aprilTagProcessor == null || visionPortal == null) {
            currentDetection = null;
            tagDetected = false;
            return;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections == null || detections.isEmpty()) {
            currentDetection = null;
            tagDetected = false;
        } else {
            // Podrías filtrar por ID específico aquí si es necesario
            currentDetection = detections.get(0);
            tagDetected = true;
        }
    }

    /**
     * Control mejorado basado en AprilTag
     */
    private void controlMecanumBasedOnTag() {
        if (!tagDetected || currentDetection == null) {
            stopAllMotors();
            currentState = MovementState.STOP;
            return;
        }

        double bearing = currentDetection.ftcPose.bearing;
        double range = currentDetection.ftcPose.range;

        // Primero, verificar si estamos muy cerca (detener)
        if (range < MINIMUM_RANGE) {
            stopAllMotors();
            currentState = MovementState.STOP;
            telemetry.addData("Estado", "¡TAG ALCANZADO!");
            return;
        }

        // Control PID simple para alineación
        double strafeCorrection = 0;

        // Alinear usando bearing
        if (Math.abs(bearing) > BEARING_THRESHOLD) {
            strafeCorrection = bearing * STRAFE_KP;
            // Limitar la corrección
            strafeCorrection = Math.max(-STRAFE_POWER, Math.min(STRAFE_POWER, strafeCorrection));
        }

        // Control de avance basado en distancia
        double forwardPower = 0;
        if (range > MAX_RANGE) {
            forwardPower = 0.2; // Avanzar lentamente si está lejos
        }

        // Aplicar movimiento combinado (strafe + forward)
        if (strafeCorrection != 0 || forwardPower != 0) {
            mecanumDrive(strafeCorrection, forwardPower, 0);
            updateMovementState(strafeCorrection, forwardPower);
        } else {
            stopAllMotors();
            currentState = MovementState.STOP;
        }
    }

    /**
     * Control mecanum completo
     */
    private void mecanumDrive(double strafe, double forward, double rotate) {
        if (leftFront == null || leftBack == null || rightFront == null || rightBack == null) return;

        // Fórmula mecanum
        double leftFrontPower = forward + strafe + rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double rightBackPower = forward + strafe - rotate;

        // Normalizar
        double max = Math.max(1.0, Math.abs(leftFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        leftFrontPower /= max;
        leftBackPower /= max;
        rightFrontPower /= max;
        rightBackPower /= max;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    /**
     * Actualiza el estado de movimiento
     */
    private void updateMovementState(double strafe, double forward) {
        if (Math.abs(strafe) > 0.1) {
            currentState = strafe < 0 ? MovementState.STRAFE_LEFT : MovementState.STRAFE_RIGHT;
        } else if (Math.abs(forward) > 0.1) {
            currentState = forward > 0 ? MovementState.FORWARD : MovementState.BACKWARD;
        } else {
            currentState = MovementState.STOP;
        }
    }

    /**
     * Controles de emergencia
     */
    private boolean handleEmergencyControls() {
        if (gamepad1.a) {
            stopAllMotors();
            telemetry.addData("EMERGENCIA", "Motores detenidos");
            return true;
        }

        if (gamepad1.b) {
            // Resetear estado
            currentState = MovementState.STOP;
            telemetry.addData("Reset", "Estado reiniciado");
        }

        if (gamepad1.x && visionPortal != null) {
            visionPortal.stopStreaming();
        }

        if (gamepad1.y && visionPortal != null) {
            visionPortal.resumeStreaming();
        }

        return false;
    }

    /**
     * Strafe izquierdo
     */
    private void strafeLeft() {
        mecanumDrive(-STRAFE_POWER, 0, 0);
        currentState = MovementState.STRAFE_LEFT;
    }

    /**
     * Strafe derecho
     */
    private void strafeRight() {
        mecanumDrive(STRAFE_POWER, 0, 0);
        currentState = MovementState.STRAFE_RIGHT;
    }

    /**
     * Detiene todos los motores
     */
    private void stopAllMotors() {
        if (leftFront != null) leftFront.setPower(0);
        if (leftBack != null) leftBack.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
        if (rightBack != null) rightBack.setPower(0);
    }

    /**
     * Muestra información en telemetría
     */
    private void displayTelemetry() {
        telemetry.clearAll();

        telemetry.addLine("=== SEGUIMIENTO APRILTAG ===");

        if (tagDetected && currentDetection != null) {
            telemetry.addData("✓ Tag ID", currentDetection.id);
            telemetry.addData("Bearing", "%.1f°", currentDetection.ftcPose.bearing);
            telemetry.addData("Range", "%.1f in", currentDetection.ftcPose.range);
            telemetry.addData("Yaw", "%.1f°", currentDetection.ftcPose.yaw);

            // Flecha indicadora
            String arrow = getDirectionArrow(currentDetection.ftcPose.bearing);
            telemetry.addData("Dirección", arrow);

            telemetry.addData("Estado", currentState.toString());

            // Potencias
            if (leftFront != null) {
                telemetry.addLine("\nPotencias Motor:");
                telemetry.addData("LF", "%.2f | RF: %.2f",
                        leftFront.getPower(), rightFront.getPower());
                telemetry.addData("LB", "%.2f | RB: %.2f",
                        leftBack.getPower(), rightBack.getPower());
            }
        } else {
            telemetry.addData("✗ Estado", "BUSCANDO TAG...");
            telemetry.addData("Movimiento", "DETENIDO");
        }

        telemetry.addLine("\n=== CONTROLES ===");
        telemetry.addData("A", "Parada emergencia");
        telemetry.addData("B", "Reset estado");
        telemetry.addData("X/Y", "Pausar/Reanudar cámara");
    }

    /**
     * Obtiene una flecha indicadora de dirección
     */
    private String getDirectionArrow(double bearing) {
        if (bearing < -BEARING_THRESHOLD) {
            return "←←← IZQUIERDA ←←←";
        } else if (bearing > BEARING_THRESHOLD) {
            return "→→→ DERECHA →→→";
        } else {
            return "↑↑↑ CENTRADO ↑↑↑";
        }
    }

    /**
     * Limpia recursos
     */
    private void cleanup() {
        stopAllMotors();

        if (visionPortal != null) {
            visionPortal.close();
        }

        telemetry.addData("Estado", "PROGRAMA FINALIZADO");
        telemetry.update();
    }
}