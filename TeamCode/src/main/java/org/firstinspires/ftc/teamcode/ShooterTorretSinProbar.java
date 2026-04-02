package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Torreta AprilTag Mejorada")
public class ShooterTorretSinProbar extends LinearOpMode {

    // ============ MOTORES Y SERVOS ============
    private DcMotor torret;          // Motor de rotación horizontal
    private DcMotor shooter;          // Motor del shooter (opcional)
    private Servo tiltServo;           // Servo para inclinación (opcional)
    private Servo triggerServo;         // Servo para disparar (opcional)

    // ============ CONSTANTES DE CONFIGURACIÓN ============
    private static final String TORRET_MOTOR_NAME = "Torret";
    private static final String SHOOTER_MOTOR_NAME = "Shooter";  // Opcional
    private static final String TILT_SERVO_NAME = "tilt";        // Opcional
    private static final String TRIGGER_SERVO_NAME = "trigger";  // Opcional

    private static final boolean USE_WEBCAM = true;
    private static final String WEBCAM_NAME = "Webcam 1";

    // ============ CONSTANTES DE CONTROL ============
    private static final double BEARING_THRESHOLD = 2.0;      // Grados de tolerancia
    private static final double TORRET_SPEED = 0.15;          // Velocidad base de la torreta
    private static final double TORRET_MAX_SPEED = 0.3;       // Velocidad máxima
    private static final double TORRET_MIN_SPEED = 0.08;      // Velocidad mínima

    // Control PID simplificado
    private static final double TORRET_KP = 0.03;             // Ganancia proporcional

    // ============ POSICIONES DE SERVOS ============
    private static final double TILT_UP_POSITION = 0.7;        // Posición para apuntar arriba
    private static final double TILT_DOWN_POSITION = 0.3;      // Posición para apuntar abajo
    private static final double TILT_MID_POSITION = 0.5;       // Posición media
    private static final double TRIGGER_RETRACTED = 0.3;       // Servo retraído (listo)
    private static final double TRIGGER_EXTENDED = 0.7;        // Servo extendido (disparar)

    // ============ VARIABLES DE VISIÓN ============
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private AprilTagDetection currentDetection = null;
    private boolean tagDetected = false;

    // ============ VARIABLES DE CONTROL ============
    private double targetBearing = 0;
    private boolean autoAimEnabled = true;
    private boolean shooterEnabled = false;
    private int targetTagId = -1;  // -1 significa cualquier tag

    @Override
    public void runOpMode() {
        // Inicializar hardware
        initializeHardware();

        // Inicializar AprilTag
        initializeAprilTag();

        // Configuración inicial de servos
        if (tiltServo != null) {
            tiltServo.setPosition(TILT_MID_POSITION);
        }

        if (triggerServo != null) {
            triggerServo.setPosition(TRIGGER_RETRACTED);
        }

        telemetry.addData("Estado", "Inicializado. Esperando START...");
        telemetry.addData("Controles", "A=AutoAim ON/OFF, B=Disparar, X=Pausar cámara, Y=Reanudar");
        telemetry.addData("D-pad", "UP/DOWN=Seleccionar tag, LEFT/RIGHT=Ajuste manual");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Actualizar detección
            updateCurrentDetection();

            // Manejar controles del gamepad
            handleGamepadControls();

            // Control automático de la torreta
            if (autoAimEnabled) {
                controlTorretWithAprilTag();
            } else {
                // Control manual con D-pad LEFT/RIGHT
                controlTorretManual();
            }

            // Mostrar telemetría
            displayTelemetry();

            sleep(20);
        }

        // Limpiar recursos
        cleanup();
    }

    /**
     * Inicializa todos los componentes de hardware
     */
    private void initializeHardware() {
        try {
            // Motor de torreta (obligatorio)
            torret = hardwareMap.get(DcMotor.class, TORRET_MOTOR_NAME);
            torret.setDirection(DcMotor.Direction.FORWARD);
            torret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            torret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Motor shooter (opcional)
            try {
                shooter = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR_NAME);
                shooter.setDirection(DcMotor.Direction.FORWARD);
                shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } catch (Exception e) {
                shooter = null;
                telemetry.addData("Info", "Shooter no encontrado");
            }

            // Servo tilt (opcional)
            try {
                tiltServo = hardwareMap.get(Servo.class, TILT_SERVO_NAME);
            } catch (Exception e) {
                tiltServo = null;
            }

            // Servo trigger (opcional)
            try {
                triggerServo = hardwareMap.get(Servo.class, TRIGGER_SERVO_NAME);
            } catch (Exception e) {
                triggerServo = null;
            }

            telemetry.addData("Hardware", "Inicializado correctamente");

        } catch (Exception e) {
            telemetry.addData("ERROR Hardware", e.getMessage());
        }
    }

    /**
     * Inicializa el sistema de AprilTag
     */
    private void initializeAprilTag() {
        try {
            // Configurar procesador AprilTag
            AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder();
            builder.setDrawAxes(true);
            builder.setDrawCubeProjection(true);
            builder.setDrawTagOutline(true);
            builder.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);

            aprilTagProcessor = builder.build();

            // Configurar portal de visión
            VisionPortal.Builder visionBuilder = new VisionPortal.Builder();

            if (USE_WEBCAM) {
                WebcamName webcam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
                visionBuilder.setCamera(webcam);
            } else {
                visionBuilder.setCamera(BuiltinCameraDirection.BACK);
            }

            visionBuilder.setCameraResolution(new android.util.Size(640, 480));
            visionBuilder.addProcessor(aprilTagProcessor);

            visionPortal = visionBuilder.build();

            telemetry.addData("Visión", "AprilTag inicializado");

        } catch (Exception e) {
            telemetry.addData("ERROR Visión", e.getMessage());
        }
    }

    /**
     * Actualiza la detección actual del AprilTag
     */
    private void updateCurrentDetection() {
        if (aprilTagProcessor == null) {
            currentDetection = null;
            tagDetected = false;
            return;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections == null || detections.isEmpty()) {
            currentDetection = null;
            tagDetected = false;
        } else {
            // Si hay un ID específico seleccionado, buscar ese
            if (targetTagId > 0) {
                currentDetection = findTagById(detections, targetTagId);
            } else {
                // Si no, tomar el primer tag detectado
                currentDetection = detections.get(0);
            }
            tagDetected = (currentDetection != null);
        }
    }

    /**
     * Busca un tag específico por ID
     */
    private AprilTagDetection findTagById(List<AprilTagDetection> detections, int targetId) {
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Control automático de la torreta basado en AprilTag
     */
    private void controlTorretWithAprilTag() {
        if (torret == null) return;

        if (tagDetected && currentDetection != null) {
            double bearing = currentDetection.ftcPose.bearing;
            targetBearing = bearing;

            // Control PID simple
            double error = bearing;
            double power = error * TORRET_KP;

            // Limitar velocidad
            if (Math.abs(power) > TORRET_MAX_SPEED) {
                power = Math.signum(power) * TORRET_MAX_SPEED;
            } else if (Math.abs(power) < TORRET_MIN_SPEED && Math.abs(error) > BEARING_THRESHOLD) {
                power = Math.signum(power) * TORRET_MIN_SPEED;
            }

            // Aplicar potencia si está fuera del umbral
            if (Math.abs(error) > BEARING_THRESHOLD) {
                torret.setPower(power);
            } else {
                torret.setPower(0);

                // Si estamos centrados y el shooter está encendido, ajustar tilt
                adjustTiltBasedOnRange();
            }

        } else {
            torret.setPower(0);
        }
    }

    /**
     * Ajusta el tilt basado en la distancia
     */
    private void adjustTiltBasedOnRange() {
        if (tiltServo == null || currentDetection == null) return;

        double range = currentDetection.ftcPose.range;

        // Lógica simple: más lejos = apuntar más arriba
        if (range < 24) { // Menos de 24 pulgadas
            tiltServo.setPosition(TILT_DOWN_POSITION);
        } else if (range > 48) { // Más de 48 pulgadas
            tiltServo.setPosition(TILT_UP_POSITION);
        } else {
            // Interpolación lineal entre 24 y 48 pulgadas
            double t = (range - 24) / 24;
            double position = TILT_DOWN_POSITION + t * (TILT_UP_POSITION - TILT_DOWN_POSITION);
            tiltServo.setPosition(position);
        }
    }

    /**
     * Control manual con D-pad
     */
    private void controlTorretManual() {
        if (torret == null) return;

        if (gamepad1.dpad_left) {
            torret.setPower(-TORRET_SPEED); // Girar izquierda
        } else if (gamepad1.dpad_right) {
            torret.setPower(TORRET_SPEED);  // Girar derecha
        } else {
            torret.setPower(0);
        }
    }

    /**
     * Maneja todos los controles del gamepad
     */
    private void handleGamepadControls() {
        // Activar/desactivar auto-aim con A
        if (gamepad1.a && gamepad1.a != gamepad1.a) { // Detectar flanco de subida
            autoAimEnabled = !autoAimEnabled;
            if (!autoAimEnabled && torret != null) {
                torret.setPower(0);
            }
        }

        // Control de shooter (botón B)
        if (gamepad1.b) {
            fire();
        }

        // Control de cámara con X/Y
        if (gamepad1.x && visionPortal != null) {
            visionPortal.stopStreaming();
        }
        if (gamepad1.y && visionPortal != null) {
            visionPortal.resumeStreaming();
        }

        // Selección de tag con D-pad UP/DOWN
        if (gamepad1.dpad_up && gamepad1.dpad_up != gamepad1.dpad_up) {
            targetTagId++;
            if (targetTagId > 10) targetTagId = -1;
        }
        if (gamepad1.dpad_down && gamepad1.dpad_down != gamepad1.dpad_down) {
            targetTagId--;
            if (targetTagId < -1) targetTagId = 10;
        }

        // Control shooter (gatillos)
        if (shooter != null) {
            if (gamepad1.left_trigger > 0.1) {
                shooter.setPower(gamepad1.left_trigger);
                shooterEnabled = true;
            } else if (gamepad1.right_trigger > 0.1) {
                shooter.setPower(-gamepad1.right_trigger);
                shooterEnabled = true;
            } else if (!shooterEnabled) {
                shooter.setPower(0);
            }
        }
    }

    /**
     * Secuencia de disparo
     */
    private void fire() {
        if (triggerServo == null) return;

        // Verificar que estamos apuntando a un tag
        if (!tagDetected) {
            telemetry.addData("Disparo", "¡No hay tag detectado!");
            return;
        }

        // Secuencia de disparo
        triggerServo.setPosition(TRIGGER_EXTENDED);
        sleep(200); // Esperar 200ms
        triggerServo.setPosition(TRIGGER_RETRACTED);

        telemetry.addData("Disparo", "¡FIRE!");
    }

    /**
     * Muestra toda la información en telemetría
     */
    private void displayTelemetry() {
        telemetry.clearAll();

        telemetry.addLine("=== TORRETA APRILTAG ===");

        // Estado del sistema
        telemetry.addData("AutoAim", autoAimEnabled ? "ACTIVADO ✓" : "DESACTIVADO ✗");
        telemetry.addData("Target Tag ID", targetTagId == -1 ? "CUALQUIERA" : targetTagId);

        // Información del tag
        if (tagDetected && currentDetection != null) {
            telemetry.addLine("\n✓ TAG DETECTADO:");
            telemetry.addData("ID", currentDetection.id);
            telemetry.addData("Bearing", String.format("%.1f°", currentDetection.ftcPose.bearing));
            telemetry.addData("Range", String.format("%.1f in", currentDetection.ftcPose.range));
            telemetry.addData("Yaw", String.format("%.1f°", currentDetection.ftcPose.yaw));

            // Indicador visual de dirección
            String arrow = getDirectionArrow(currentDetection.ftcPose.bearing);
            telemetry.addData("Alineación", arrow);

            // Información del tag
            if (currentDetection.metadata != null) {
                telemetry.addData("Nombre", currentDetection.metadata.name);
            }
        } else {
            telemetry.addLine("\n✗ BUSCANDO TAG...");
        }

        // Estado de la torreta
        if (torret != null) {
            telemetry.addData("\nPotencia Torreta", String.format("%.2f", torret.getPower()));
        }

        // Estado del shooter
        if (shooter != null) {
            telemetry.addData("Potencia Shooter", String.format("%.2f", shooter.getPower()));
        }

        // Controles
        telemetry.addLine("\n=== CONTROLES ===");
        telemetry.addData("A", "AutoAim ON/OFF");
        telemetry.addData("B", "Disparar");
        telemetry.addData("X/Y", "Pausar/Reanudar cámara");
        telemetry.addData("D-pad", "UP/DOWN=Seleccionar ID, LEFT/RIGHT=Manual");
        telemetry.addData("Gatillos", "Control shooter");
    }

    /**
     * Genera una flecha indicadora de dirección
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
     * Limpia recursos al finalizar
     */
    private void cleanup() {
        if (torret != null) torret.setPower(0);
        if (shooter != null) shooter.setPower(0);

        if (visionPortal != null) {
            visionPortal.close();
        }

        telemetry.addData("Estado", "PROGRAMA FINALIZADO");
        telemetry.update();
    }
}