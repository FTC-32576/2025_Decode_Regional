package ftc.team.allmight.plusultra.teamcode.auto;

import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import ftc.team.Java_Is_AllMight.Sensors.LimeMight;
import ftc.team.allmight.plusultra.teamcode.roadrunner.drive.SampleTankDrive;
import ftc.team.allmight.plusultra.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import ftc.team.allmight.plusultra.teamcode.utils.AprilTagPatternUtil;


@Autonomous(name="Identificar Padrão")
public class DetectPatternAuto extends LinearOpMode {





    private AprilTagPatternUtil patternUtil;
    private LimeMight limeMight;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(50.54, 1.23, Math.toRadians(243.43)))
                .splineTo(new Vector2d(44.35, 51.18), Math.toRadians(227.07))
                .lineTo(new Vector2d(16.49, -34.75))
                .build();

        limeMight = new LimeMight(hardwareMap,
                "limelight",
                "imu",
                RevHubOrientationOnRobot.UsbFacingDirection.UP,
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                null,
                null
        );

        patternUtil = new AprilTagPatternUtil(limeMight);
        waitForStart();

        AprilTagPatternUtil.Pattern pattern = patternUtil.detectPattern(telemetry);
        telemetry.addData("Detected Patern", pattern);
        telemetry.update();
        drive.followTrajectorySequence(trajectory0);

        while (opModeIsActive()){
            drive.update();

            idle();
        }
    }
}
