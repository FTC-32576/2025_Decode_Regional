package ftc.team.allmight.plusultra.teamcode.auto;

import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Pathing.PlusPathing;
import ftc.team.Java_Is_AllMight.Pathing.PlusTrajectory;
import ftc.team.Java_Is_AllMight.Pathing.PlusTrajectoryBuilder;
import ftc.team.Java_Is_AllMight.Pathing.PlusTrajectoryFollower;
import ftc.team.Java_Is_AllMight.Sensors.LimeMight;
import ftc.team.allmight.plusultra.teamcode.roadrunner.drive.SampleTankDrive;
import ftc.team.allmight.plusultra.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import ftc.team.allmight.plusultra.teamcode.utils.AprilTagPatternUtil;

@Autonomous(name = "Detect Pattern (Webcam)")
public class DetectPatternAuto extends LinearOpMode {

    private AprilTagPatternUtil patternUtil;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private PlusPathing pathing = new PlusPathing(new PIDConfig(0,0,0), new PIDConfig(0,0,0));

    private SampleTankDrive drive;
    PlusTrajectory trajectory  = new PlusTrajectoryBuilder(drive.getPoseEstimate())
            .toWaypoint(50,30, Math.toRadians(90))
            .build();

    PlusTrajectoryFollower follower = new PlusTrajectoryFollower(pathing);


    @Override
    public void runOpMode() {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        // Inicializa a pipeline de AprilTags
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        patternUtil = new AprilTagPatternUtil(visionPortal, aprilTag);

        follower.follow(trajectory);

        drive.update();

        Pose2d pose = drive.getPoseEstimate();

        PlusPathing.ControlOutput out = follower.update(pose);

        waitForStart();

        double left = out.drive - out.turn;
        double right = out.drive + out.turn;

        drive.setMotorPowers(left, right);

        AprilTagPatternUtil.Pattern pattern = patternUtil.detectPattern(telemetry);
        telemetry.addData("Detected Pattern", pattern);
        telemetry.update();

        while (opModeIsActive()) {
            drive.update();
            idle();
        }
    }
}
