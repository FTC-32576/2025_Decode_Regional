package ftc.team.allmight.plusultra.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import ftc.team.Java_Is_AllMight.Sensors.AprilTagWebcam;
import ftc.team.Java_Is_AllMight.Utils.Alliance;
import ftc.team.allmight.plusultra.teamcode.subsystems.Drive;
import ftc.team.allmight.plusultra.teamcode.subsystems.ServoSubsystem;
import ftc.team.allmight.plusultra.teamcode.subsystems.ShooterSubsystem;
import ftc.team.allmight.plusultra.teamcode.utils.MathUtils;

@TeleOp(name = "ShooterTestWebcam REAL")
public class ShooterTestWebcam extends OpMode {

    private ShooterSubsystem shooter;
    private ServoSubsystem servo;
    private Drive drive;

    private AprilTagWebcam webcam;
    private Alliance alliance = Alliance.RED;

    private double lastDistance = 0;
    private double lastPower = 0;

    @Override
    public void init() {
        shooter = new ShooterSubsystem();
        shooter.init(hardwareMap);

        servo = new ServoSubsystem(hardwareMap);
        drive = new Drive(hardwareMap);

        webcam = new AprilTagWebcam();
        webcam.init(hardwareMap, telemetry);

        telemetry.addLine("iniciado!");
    }

    @Override
    public void loop() {
        shooter.update(telemetry);

        List<AprilTagDetection> detections = webcam.aprilTagProcessorDosNgc.getDetections();
        AprilTagDetection target = findAllianceTag(detections, alliance.getTagID());

        if (target != null) {
            double distanceM = target.ftcPose.range;
            lastDistance = distanceM;

            double power = MathUtils.calculateShooterPower(distanceM);
            lastPower = power;

            shooter.setPower(power);

            if (shooter.isReadyToShoot()) {
                servo.setPosition(0.345);
            } else {
                servo.setPosition(0.0);
            }

            telemetry.addLine("=== TAG DETECTADA ===");
            telemetry.addData("Distância (m)", String.format("%.2f", distanceM));
            telemetry.addData("Power calculado", String.format("%.2f", power));
            telemetry.addData("Shooter RPM", shooter.getRPM());
            telemetry.addData("Spin-up (s)", shooter.getSpinupTimeSeconds());
        } else {
            telemetry.addLine("Nenhuma tag da aliança detectada");
            servo.setPosition(0.0);
        }

        drive.drive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.update();
    }

    private AprilTagDetection findAllianceTag(List<AprilTagDetection> detections, int tagId) {
        if (detections == null) return null;
        for (AprilTagDetection tag : detections) {
            if (tag.id == tagId) return tag;
        }
        return null;
    }
}