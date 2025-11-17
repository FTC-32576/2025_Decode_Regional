package ftc.team.allmight.plusultra.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import ftc.team.Java_Is_AllMight.Sensors.CameraMight;
import ftc.team.Java_Is_AllMight.Utils.Alliance;
import ftc.team.allmight.plusultra.teamcode.roadrunner.drive.SampleTankDrive;
import ftc.team.allmight.plusultra.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import ftc.team.allmight.plusultra.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import ftc.team.allmight.plusultra.teamcode.utils.AprilTagPatternUtil;
import ftc.team.allmight.plusultra.teamcode.utils.MathUtils;

@TeleOp(name = "Detect Pattern (Webcam)")
public class DetectPatternAuto extends LinearOpMode {

    private AprilTagPatternUtil patternUtil;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private AprilTagProcessor aprilTagProcessor;

    private Servo servo;

    private Alliance alliance = Alliance.RED;

    CameraMight cameraMight = new CameraMight();

    private SampleTankDrive drive;
    @Override
    public void runOpMode() {
        drive = new SampleTankDrive(hardwareMap);
        servo = hardwareMap.get(Servo.class, "servidor");
        cameraMight.init(hardwareMap, telemetry);
        patternUtil = new AprilTagPatternUtil(cameraMight.visionPortalDoNgc, cameraMight.aprilTagProcessorDosNgc);

        aprilTagProcessor = cameraMight.aprilTagProcessorDosNgc;
        telemetry.addData("Aliança atual", alliance.name());
        telemetry.addData("Tag ID alvo", alliance.getTagID());
        telemetry.update();

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        waitForStart();

        AprilTagPatternUtil.Pattern pattern;

        if (isStopRequested()) return;


        List<AprilTagDetection>  detections = cameraMight.aprilTagProcessorDosNgc.getDetections();
        AprilTagDetection targetTag = getAllianceTag(detections, alliance.getTagID());


        if (targetTag == null) {
            telemetry.addLine("Nenhuma tag detectada!");
            telemetry.update();
            return;
        }

        // pegar as Poses - *100 pra converter pra cm, só lembra disos
        double x = targetTag.ftcPose.x * 100;
        double y = targetTag.ftcPose.y * 100;
        double z = targetTag.ftcPose.z * 100;
        double distancia = targetTag.ftcPose.range * 100;
        double yaw = targetTag.ftcPose.yaw;

        telemetry.addData("Distância até a tag (cm)", distancia);
        telemetry.addData("Offset X (cm)", x);
        telemetry.addData("Offset Y (cm)", y);
        telemetry.addData("Yaw (°)", yaw);

        // Calcula ângulo e distância pro robô
        // Considera X = frente da câmera, Y = esquerda/direita
        double anguloRobo = Math.atan2(y, x);
        double distanciaRobo = Math.hypot(x, y);

        // Quer parar 20cm antes da tag
        double distanciaFinal = distanciaRobo - 20;
        if (distanciaFinal < 0) distanciaFinal = 0;

        // 4️⃣ Gera trajetória
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .turn(anguloRobo)         // gira na direção da tag
                .forward(distanciaFinal)  // avança até chegar perto
                .build();

        telemetry.addLine("Seguindo trajetória até a tag...");
        telemetry.update();

        // 5️⃣ Executa
        drive.followTrajectorySequence(traj);

        telemetry.addLine("Chegou no ponto de destino!");
        telemetry.update();

            if(targetTag != null){
                double distanceMeters = targetTag.ftcPose.range;
                double shooterPower = MathUtils.calculateShooterPower(distanceMeters);

                Pose2d cameraPose = new Pose2d(targetTag.ftcPose.x, targetTag.ftcPose.y, Math.toRadians(targetTag.ftcPose.yaw));

                telemetry.addLine("==== TAG DETECTADA ====");
                telemetry.addData("Alliance", alliance.name());
                telemetry.addData("Tag ID", alliance.getTagID());
                telemetry.addData("Distância (m)", "%.2f", distanceMeters);
                telemetry.addData("Shooter Power", "%.2f", shooterPower);
                telemetry.addData("Posição X (cm)", "%.2f", targetTag.ftcPose.x);
                telemetry.addData("Posição Y (cm)", "%.2f", targetTag.ftcPose.y);
                telemetry.addData("Heading (°)", "%.2f", targetTag.ftcPose.yaw);
                telemetry.addLine();

            } else{
                telemetry.addLine("Nenhuma tag da aliança detectada.");
            }
            pattern = patternUtil.detectPattern(telemetry);


            if(gamepad1.a){
                servo.setPosition(1.0);
            } else if (gamepad1.b) {
                servo.setPosition(0.0);
            }

    }

    /**
     * Procura a tag correspondente à aliança.
     */
    private AprilTagDetection getAllianceTag(List<AprilTagDetection> detections, int allianceTagID) {
        for (AprilTagDetection tag : detections) {
            if (tag.id == allianceTagID) {
                return tag;
            }
        }
        return null;
    }

}
