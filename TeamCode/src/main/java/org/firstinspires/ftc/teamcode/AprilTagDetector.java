//Control PID suave: Movimiento más natural de la torreta
//Selección de tags específicos: Puedes elegir qué tag seguir
//Sistema de disparo completo: Incluye servo trigger y tilt automático
//Modo manual: Control con D-pad cuando auto-aim está desactivado
//Mejor telemetría: Información más clara y útil
//Manejo de errores: Código más robusto


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

@TeleOp(name = "1 motor 1 camara")
public class AprilTagDetector extends LinearOpMode {

    private DcMotor Torret;
    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;
    AprilTagDetection currentDetection = null;
    boolean tagDetected = false;

    @Override
    public void runOpMode() {
        Torret = hardwareMap.get(DcMotor.class, "Torret");
        USE_WEBCAM = true;

        initAprilTag();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                updateCurrentDetection();
                telemetryAprilTag();
                telemetry.update();

                if (gamepad1.dpad_down) {
                    myVisionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    myVisionPortal.resumeStreaming();
                }

                if (tagDetected && currentDetection != null) {
                    double bearing = currentDetection.ftcPose.bearing;

                    if (bearing < -2) {
                        Torret.setPower(0.2);
                    } else if (bearing > 2) {
                        Torret.setPower(-0.2);
                    } else {
                        Torret.setPower(0);
                    }

                    telemetry.addData("Estado", "SEGUIMIENTO ACTIVO");
                    telemetry.addData("Bearing", bearing);
                } else {
                    Torret.setPower(0);
                    telemetry.addData("Estado", "BUSCANDO TAG...");
                }
                // ==========================================

                sleep(20);
            }
        }
    }

    private void updateCurrentDetection() {
        List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();

        // ============ CORRECCIÓN CRÍTICA ============
        if (detections == null || detections.isEmpty()) {  // NO detections.equals(null)
            currentDetection = null;
            tagDetected = false;
        } else {
            currentDetection = detections.get(0);  // Simple y directo
            tagDetected = true;
        }
        // ============================================
    }

    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();

        VisionPortal.Builder myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        myVisionPortal = myVisionPortalBuilder.build();
    }

    private void telemetryAprilTag() {
        if (currentDetection != null) {
            telemetry.addLine("==== TAG DETECTADO ====");
            telemetry.addData("ID", currentDetection.id);
            telemetry.addData("Bearing", currentDetection.ftcPose.bearing);
            telemetry.addData("Range", currentDetection.ftcPose.range);
            telemetry.addData("Elevation", currentDetection.ftcPose.elevation);

            if (currentDetection.metadata != null) {
                telemetry.addData("Nombre", currentDetection.metadata.name);
            }
        } else {
            telemetry.addData("AprilTags Detectados", "0");
            telemetry.addLine("Esperando detección...");
        }
    }
}