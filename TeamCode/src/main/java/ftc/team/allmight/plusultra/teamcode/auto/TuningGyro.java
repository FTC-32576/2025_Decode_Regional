package ftc.team.allmight.plusultra.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import ftc.team.Java_Is_AllMight.Sensors.IMUMight;
import ftc.team.allmight.plusultra.teamcode.subsystems.Drive;

//@Autonomous(name = "Tuning Gyro")
public class TuningGyro extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IMUMight imu = new IMUMight(hardwareMap, "imu", RevHubOrientationOnRobot.UsbFacingDirection.UP, RevHubOrientationOnRobot.LogoFacingDirection.RIGHT);
        imu.resetYaw();

        Drive drive = new Drive(hardwareMap, imu);

        waitForStart();

        drive.turnIMU(90, 0.55, 5, telemetry);
    }
}
