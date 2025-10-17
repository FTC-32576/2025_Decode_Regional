package ftc.team.allmight.plusultra.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import ftc.team.Java_Is_AllMight.Sensors.LimeMight;
import ftc.team.allmight.plusultra.teamcode.utils.AprilTagPatternUtil;


@Autonomous(name="Identificar Padrão")
public class DetectPatternAuto extends LinearOpMode {

    private AprilTagPatternUtil patternUtil;
    private LimeMight limeMight;

    @Override
    public void runOpMode() throws InterruptedException {

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

        while (opModeIsActive()){
            idle();
        }
    }
}
