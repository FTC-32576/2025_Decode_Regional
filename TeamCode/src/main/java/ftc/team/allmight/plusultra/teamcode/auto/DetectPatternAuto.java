package ftc.team.allmight.plusultra.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import ftc.team.Java_Is_AllMight.Sensors.AprilTagWebcam;
import ftc.team.Java_Is_AllMight.Utils.Alliance;
import ftc.team.allmight.plusultra.teamcode.roadrunner.drive.SampleTankDrive;
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

    AprilTagWebcam aprilTagWebcam  = new AprilTagWebcam();

    @Override
    public void runOpMode() {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        servo = hardwareMap.get(Servo.class, "servidor");


        aprilTagWebcam.init(hardwareMap, telemetry);

        patternUtil = new AprilTagPatternUtil(aprilTagWebcam.visionPortalDoNgc, aprilTagWebcam.aprilTagProcessorDosNgc);

        aprilTagProcessor = aprilTagWebcam.aprilTagProcessorDosNgc;
        telemetry.addLine("🚀 Shooter Distance Logger pronto!");
        telemetry.addData("Aliança atual", alliance.name());
        telemetry.addData("Tag ID alvo", alliance.getTagID());
        telemetry.update();

        waitForStart();

        AprilTagPatternUtil.Pattern pattern;

        while (opModeIsActive()) {
            drive.update();
            List<AprilTagDetection>  detections = aprilTagWebcam.aprilTagProcessorDosNgc.getDetections();

            AprilTagDetection targetTag = getAllianceTag(detections, alliance.getTagID());
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
