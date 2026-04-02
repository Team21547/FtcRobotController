package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Torreta AprilTag PID")
public class AprilTagDetectorPID extends LinearOpMode {

    // ============ MOTOR ============
    private DcMotor torret;

    // ============ VARIABLES DE VISIÓN ============
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private AprilTagDetection currentDetection = null;
    private boolean tagDetected = false;
    private boolean useWebcam = true;

    // ============ CONSTANTES DE CONTROL ============
    private static final double BEARING_THRESHOLD = 2.0;      // Grados de tolerancia
    private static final double MAX_POWER = 0.3;              // Potencia máxima del motor
    private static final double MIN_POWER = 0.08;              // Potencia mínima para movimiento

    // ============ VARIABLES PID ============
    private double kP = 0.015;          // Ganancia proporcional (ajustable)
    private double kI = 0.0005;           // Ganancia integral
    private double kD = 0.001;          // Ganancia derivativa

    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime = 0;

    // ============ SELECCIÓN DE TAGS ============
    private int targetTagId = -1;        // -1 significa "cualquier tag"
    private String targetTagName = "CUALQUIERA";

    // ============ CONTROL DE MODO ============
    private boolean autoAimEnabled = true;
    private boolean cameraPaused = false;

    @Override
    public void runOpMode() {
        // Inicializar hardware con manejo de errores
        if (!initializeHardware()) {
            telemetry.addData("ERROR", "No se pudo inicializar el hardware");
            telemetry.update();
            return;
        }

        // Inicializar visión con manejo de errores
        if (!initializeAprilTag()) {
            telemetry.addData("ERROR", "No se pudo inicializar la cámara");
            telemetry.update();
            return;
        }

        // Mostrar instrucciones
        showInstructions();

        waitForStart();

        // Resetear PID al iniciar
        resetPID();

        while (opModeIsActive()) {
            // Actualizar detección de tags
            updateCurrentDetection();

            // Manejar controles del gamepad
            handleGamepadControls();

            // Control PID de la torreta
            if (autoAimEnabled) {
                controlTorretWithPID();
            } else {
                // Modo manual (detener motor)
                torret.setPower(0);
            }

            // Mostrar telemetría mejorada
            displayEnhancedTelemetry();

            // Pequeña pausa
            sleep(20);
        }

        // Limpiar recursos
        cleanup();
    }

    /**
     * Inicializa el hardware con manejo de errores
     */
    private boolean initializeHardware() {
        try {
            torret = hardwareMap.get(DcMotor.class, "Torret");
            torret.setDirection(DcMotor.Direction.FORWARD);
            torret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            torret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("Hardware", "✓ Motor Torret inicializado");
            return true;

        } catch (Exception e) {
            telemetry.addData("ERROR Hardware", "No se encontró motor 'Torret'");
            telemetry.addData("Sugerencia", "Verifica el nombre en la configuración");
            return false;
        }
    }

    /**
     * Inicializa el sistema de AprilTag con manejo de errores
     */
    private boolean initializeAprilTag() {
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

            if (useWebcam) {
                try {
                    WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
                    visionBuilder.setCamera(webcam);
                    telemetry.addData("Cámara", "✓ Webcam encontrada");
                } catch (Exception e) {
                    telemetry.addData("Cámara", "✗ Webcam no encontrada, usando cámara trasera");
                    visionBuilder.setCamera(BuiltinCameraDirection.BACK);
                }
            } else {
                visionBuilder.setCamera(BuiltinCameraDirection.BACK);
            }

            visionBuilder.setCameraResolution(new android.util.Size(640, 480));
            visionBuilder.addProcessor(aprilTagProcessor);

            visionPortal = visionBuilder.build();

            if (visionPortal == null) {
                telemetry.addData("ERROR", "No se pudo crear VisionPortal");
                return false;
            }

            telemetry.addData("Visión", "✓ AprilTag inicializado");
            return true;

        } catch (Exception e) {
            telemetry.addData("ERROR Visión", e.getMessage());
            return false;
        }
    }

    /**
     * Muestra las instrucciones en telemetría
     */
    private void showInstructions() {
        telemetry.clear();
        telemetry.addLine("=== TORRETA APRILTAG PID ===");
        telemetry.addLine("");
        telemetry.addLine("INSTRUCCIONES:");
        telemetry.addLine("• A = Activar/Desactivar auto-aim");
        telemetry.addLine("• X = Pausar cámara");
        telemetry.addLine("• Y = Reanudar cámara");
        telemetry.addLine("• B = Reset PID");
        telemetry.addLine("• D-pad UP = Siguiente tag ID");
        telemetry.addLine("• D-pad DOWN = Tag anterior");
        telemetry.addLine("• D-pad LEFT = Disminuir kP");
        telemetry.addLine("• D-pad RIGHT = Aumentar kP");
        telemetry.addLine("");
        telemetry.addLine("Presiona START para comenzar");
        telemetry.update();
    }

    /**
     * Actualiza la detección actual con selección de tags
     */
    private void updateCurrentDetection() {
        if (aprilTagProcessor == null || visionPortal == null) {
            currentDetection = null;
            tagDetected = false;
            return;
        }

        try {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            if (detections == null || detections.isEmpty()) {
                currentDetection = null;
                tagDetected = false;
            } else {
                // Si hay un ID específico seleccionado, buscar ese
                if (targetTagId >= 0) {
                    currentDetection = findTagById(detections, targetTagId);
                    tagDetected = (currentDetection != null);

                    if (tagDetected && currentDetection.metadata != null) {
                        targetTagName = currentDetection.metadata.name;
                    }
                } else {
                    // Si no, tomar el primer tag detectado
                    currentDetection = detections.get(0);
                    tagDetected = true;

                    if (currentDetection.metadata != null) {
                        targetTagName = currentDetection.metadata.name;
                    } else {
                        targetTagName = "Tag #" + currentDetection.id;
                    }
                }
            }
        } catch (Exception e) {
            telemetry.addData("Error detección", e.getMessage());
            currentDetection = null;
            tagDetected = false;
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
     * Control PID mejorado para la torreta
     */
    private void controlTorretWithPID() {
        if (torret == null) return;

        if (!tagDetected || currentDetection == null) {
            torret.setPower(0);
            resetPID(); // Resetear PID cuando no hay tag
            return;
        }

        double bearing = currentDetection.ftcPose.bearing;

        // Calcular error (queremos bearing = 0)
        double error = bearing;

        // Tiempo actual para cálculos derivativos
        long currentTime = System.currentTimeMillis();
        double deltaTime = (lastTime == 0) ? 20 : (currentTime - lastTime);

        // Término Proporcional
        double pTerm = kP * error;

        // Término Integral (con anti-windup)
        if (Math.abs(error) > BEARING_THRESHOLD) {
            integralSum += error * deltaTime / 1000.0; // Convertir a segundos
        } else {
            integralSum = 0; // Resetear integral cuando estamos cerca
        }

        // Limitar integral para evitar windup
        double maxIntegral = MAX_POWER / kI;
        if (integralSum > maxIntegral) integralSum = maxIntegral;
        if (integralSum < -maxIntegral) integralSum = -maxIntegral;

        double iTerm = kI * integralSum;

        // Término Derivativo
        double derivative = (error - lastError) / (deltaTime / 1000.0);
        double dTerm = kD * derivative;

        // Potencia total PID
        double power = pTerm + iTerm + dTerm;

        // Limitar potencia
        if (Math.abs(power) > MAX_POWER) {
            power = Math.signum(power) * MAX_POWER;
        } else if (Math.abs(power) < MIN_POWER && Math.abs(error) > BEARING_THRESHOLD) {
            power = Math.signum(power) * MIN_POWER;
        }

        // Aplicar potencia
        if (Math.abs(error) > BEARING_THRESHOLD) {
            torret.setPower(power);
        } else {
            torret.setPower(0);
            integralSum = 0; // Resetear integral cuando estamos en posición
        }

        // Guardar valores para próxima iteración
        lastError = error;
        lastTime = currentTime;
    }

    /**
     * Resetea las variables del PID
     */
    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
    }

    /**
     * Maneja todos los controles del gamepad
     */
    private void handleGamepadControls() {
        // Activar/desactivar auto-aim con A (detectar flanco)
        if (gamepad1.a && !autoAimEnabled) {
            autoAimEnabled = true;
            telemetry.addData("AutoAim", "ACTIVADO");
        } else if (gamepad1.a && autoAimEnabled) {
            autoAimEnabled = false;
            torret.setPower(0);
            telemetry.addData("AutoAim", "DESACTIVADO");
        }

        // Control de cámara con X/Y
        if (gamepad1.x && !cameraPaused && visionPortal != null) {
            visionPortal.stopStreaming();
            cameraPaused = true;
            telemetry.addData("Cámara", "PAUSADA");
        }
        if (gamepad1.y && cameraPaused && visionPortal != null) {
            visionPortal.resumeStreaming();
            cameraPaused = false;
            telemetry.addData("Cámara", "ACTIVA");
        }

        // Reset PID con B
        if (gamepad1.b) {
            resetPID();
            telemetry.addData("PID", "RESETEADO");
        }

        // Selección de tags con D-pad UP/DOWN
        if (gamepad1.dpad_up) {
            targetTagId++;
            if (targetTagId > 10) targetTagId = -1; // Volver a "cualquier tag"
            resetPID();
            sleep(200); // Debounce
        }
        if (gamepad1.dpad_down) {
            targetTagId--;
            if (targetTagId < -1) targetTagId = 10;
            resetPID();
            sleep(200); // Debounce
        }

        // Ajuste de kP con D-pad LEFT/RIGHT
        if (gamepad1.dpad_left) {
            kP -= 0.001;
            if (kP < 0.001) kP = 0.001;
            sleep(200);
        }
        if (gamepad1.dpad_right) {
            kP += 0.001;
            if (kP > 0.05) kP = 0.05;
            sleep(200);
        }
    }

    /**
     * Telemetría mejorada con toda la información
     */
    private void displayEnhancedTelemetry() {
        telemetry.clearAll();

        telemetry.addLine("=== TORRETA APRILTAG PID ===");
        telemetry.addLine("");

        // Estado del sistema
        telemetry.addData("AutoAim", autoAimEnabled ? "✓ ACTIVADO" : "✗ DESACTIVADO");
        telemetry.addData("Cámara", cameraPaused ? "PAUSADA" : "ACTIVA");
        telemetry.addData("Tag objetivo", targetTagId == -1 ? "CUALQUIERA" : ("ID: " + targetTagId));

        telemetry.addLine("");
        telemetry.addLine("--- INFORMACIÓN DEL TAG ---");

        // Información del tag detectado
        if (tagDetected && currentDetection != null) {
            telemetry.addData("Estado", "✓ TAG DETECTADO");
            telemetry.addData("Nombre", targetTagName);
            telemetry.addData("ID", currentDetection.id);

            // Formatear bearing con signo y flecha
            String bearingSign = (currentDetection.ftcPose.bearing > 0) ? "→" : "←";
            telemetry.addData("Bearing", String.format("%s %.1f°",
                    bearingSign, Math.abs(currentDetection.ftcPose.bearing)));

            telemetry.addData("Range", String.format("%.1f in", currentDetection.ftcPose.range));
            telemetry.addData("Yaw", String.format("%.1f°", currentDetection.ftcPose.yaw));

            // Barra de progreso visual para bearing
            drawBearingBar(currentDetection.ftcPose.bearing);

        } else {
            telemetry.addData("Estado", "✗ BUSCANDO TAG...");
            if (targetTagId >= 0) {
                telemetry.addData("Buscando ID", targetTagId);
            }
        }

        telemetry.addLine("");
        telemetry.addLine("--- PARÁMETROS PID ---");
        telemetry.addData("kP", String.format("%.4f", kP));
        telemetry.addData("kI", String.format("%.4f", kI));
        telemetry.addData("kD", String.format("%.4f", kD));
        telemetry.addData("Error", String.format("%.2f°", lastError));
        telemetry.addData("Potencia", String.format("%.2f", torret.getPower()));

        telemetry.addLine("");
        telemetry.addLine("--- CONTROLES ---");
        telemetry.addLine("A = AutoAim ON/OFF");
        telemetry.addLine("B = Reset PID");
        telemetry.addLine("X/Y = Pausar/Reanudar cámara");
        telemetry.addLine("D-pad UP/DOWN = Seleccionar tag");
        telemetry.addLine("D-pad LEFT/RIGHT = Ajustar kP");

        telemetry.update();
    }

    /**
     * Dibuja una barra visual para el bearing
     */
    private void drawBearingBar(double bearing) {
        int barLength = 20;
        int center = barLength / 2;

        // Normalizar bearing a posición en la barra (-10° a 10°)
        int position = (int)((bearing / 10.0) * center);
        if (position < -center) position = -center;
        if (position > center) position = center;

        StringBuilder bar = new StringBuilder("[");
        for (int i = 0; i < barLength; i++) {
            if (i == center + position) {
                bar.append("●");
            } else if (i == center) {
                bar.append("|");
            } else {
                bar.append("-");
            }
        }
        bar.append("]");

        telemetry.addLine("Alineación: " + bar.toString());
    }

    /**
     * Limpia recursos al finalizar
     */
    private void cleanup() {
        if (torret != null) {
            torret.setPower(0);
        }

        if (visionPortal != null) {
            visionPortal.close();
        }

        telemetry.addData("Estado", "PROGRAMA FINALIZADO");
        telemetry.update();
    }
}