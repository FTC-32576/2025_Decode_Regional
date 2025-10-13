package ftc.team.allmight.plusultra.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import ftc.team.Java_Is_AllMight.Pathing.RoadRunnerHelper;
import ftc.team.Java_Is_AllMight.Sensors.IMUHelper;
import ftc.team.Java_Is_AllMight.Config.DriveConstants;
import ftc.team.allmight.plusultra.teamcode.roadrunner.TankDrive;

@Autonomous(name = "Pose Tracker Diagnóstico")
public class PoseTrackerAuto extends LinearOpMode {

    private TankDrive drive;
    private RoadRunnerHelper rrHelper;
    private IMUHelper sharedImu;

    @Override
    public void runOpMode() throws InterruptedException {

        // Inicializa DriveConstants (ajuste conforme seu bot)
        DriveConstants.initialize(3.54, 20.0, 28, 11.811);

        // Inicializa IMU
        sharedImu = new IMUHelper(
                hardwareMap,
                "imu",
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
                RevHubOrientationOnRobot.LogoFacingDirection.UP
        );
        sharedImu.resetYaw();

        Pose2d initialPose = new Pose2d(0, 0, 0);

        drive = new TankDrive(hardwareMap, initialPose);

        RoadRunnerHelper.DriveParams driveParams = new RoadRunnerHelper.DriveParams(
                DriveConstants.WHEEL_RADIUS_METERS,
                DriveConstants.IN_PER_TICK,
                DriveConstants.TRACK_WIDTH_METERS,
                0, 0
        );

        rrHelper = new RoadRunnerHelper(
                hardwareMap, "motorEsquerdo", "motorDireito", driveParams,
                sharedImu, null, initialPose
        );

        rrHelper.setPose(initialPose);
        drive.localizer.setPose(initialPose);
        drive.updatePoseEstimate();

        telemetry.addLine("Pose Tracker pronto!");
        telemetry.addData("Pose inicial", initialPose);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double dt = 0.05;

        while (opModeIsActive()) {
            rrHelper.update(dt);
            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();

            // Diagnóstico de NaN
            if (!Double.isFinite(pose.position.x) || !Double.isFinite(pose.position.y)) {
                telemetry.addLine("⚠ Pose inválida detectada (NaN)!");
                telemetry.addData("Raw pose", pose);
                telemetry.addData("Ticks E", drive.leftMotors.get(0).getCurrentPosition());
                telemetry.addData("Ticks D", drive.rightMotors.get(0).getCurrentPosition());
                telemetry.addLine("Verifique: DriveConstants, IMU orientation e nomes dos motores.");
                telemetry.update();
                sleep(500);
                continue;
            }

            telemetry.addLine("------ POSE ATUAL ------");
            telemetry.addData("X (m)", pose.position.x);
            telemetry.addData("Y (m)", pose.position.y);
//            telemetry.addData("Heading (°)", Math.toDegrees(pose.heading));

            // Mostra os ticks crus dos encoders (diagnóstico)
            telemetry.addLine("------ ENCODERS ------");
            telemetry.addData("Esquerdo (ticks)", drive.leftMotors.get(0).getCurrentPosition());
            telemetry.addData("Direito (ticks)", drive.rightMotors.get(0).getCurrentPosition());

            telemetry.update();
            sleep((long)(dt * 1000));
        }
    }
}
