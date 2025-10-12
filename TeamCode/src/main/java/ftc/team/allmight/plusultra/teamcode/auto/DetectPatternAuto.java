package ftc.team.allmight.plusultra.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import ftc.team.Java_Is_AllMight.Sensors.LimelightHelper;
import ftc.team.allmight.plusultra.teamcode.utils.AprilTagPatternUtil;

public class DetectPatternAuto extends LinearOpMode {

    private AprilTagPatternUtil patternUtil;
    private LimelightHelper limelightHelper;

    @Override
    public void runOpMode() throws InterruptedException {

        limelightHelper = new LimelightHelper(hardwareMap,
                "limelight",
                "imu",
                RevHubOrientationOnRobot.UsbFacingDirection.UP,
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                null,
                null
        );

        patternUtil = new AprilTagPatternUtil(limelightHelper);
        waitForStart();

        AprilTagPatternUtil.Pattern pattern = patternUtil.detectPattern(telemetry);
        telemetry.addData("Detected Patern", pattern);
        telemetry.update();

        while (opModeIsActive()){
            idle();
        }
    }
}
