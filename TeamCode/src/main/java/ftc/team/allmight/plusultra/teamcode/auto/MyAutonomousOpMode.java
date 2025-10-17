package ftc.team.allmight.plusultra.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

import ftc.team.Java_Is_AllMight.Pathing.RoadRunnerHelper;
import ftc.team.Java_Is_AllMight.Sensors.IMUMight;
import ftc.team.Java_Is_AllMight.Sensors.LimeMight;
import ftc.team.Java_Is_AllMight.Sensors.LimelightDriveController;
import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Config.DriveConstants;
import ftc.team.allmight.plusultra.teamcode.roadrunner.TankDrive;

@Autonomous(name = "MyAutonomousOpMode")
public class MyAutonomousOpMode extends LinearOpMode {

    private TankDrive drive;
    private LimelightDriveController llController;
    private RoadRunnerHelper rrHelper;
    private IMUMight sharedImu;
    private LimeMight sharedLimelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize DriveConstants (adjust params for your hardware)
        DriveConstants.initialize(3.54, 20.0, 28, 18.0);  // wheelDia(in), gearRatio, encoderCPR, trackWidth(in)
        telemetry.update();  // Display init logs

        if (!DriveConstants.isValid()) {
            telemetry.addLine("ERROR: Invalid DriveConstants!");
            telemetry.update();
            return;
        }

        // DriveParams from DriveConstants
        RoadRunnerHelper.DriveParams driveParams = new RoadRunnerHelper.DriveParams(
                DriveConstants.WHEEL_RADIUS_METERS,
                DriveConstants.IN_PER_TICK,
                DriveConstants.TRACK_WIDTH_METERS,
                0,  // leftOffset
                0   // rightOffset
        );

        // Initialize shared sensors
        sharedImu = new IMUMight(
                hardwareMap, "imu",
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
                RevHubOrientationOnRobot.LogoFacingDirection.UP
        );
        sharedLimelight = new LimeMight(
                hardwareMap, "limelight", "imu",
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                new PIDConfig(0.03, 0, 0),  // angle PID
                new PIDConfig(0.05, 0, 0)   // distance PID
        );

        // Initialize TankDrive and sync with DriveConstants
        Pose2d initialPose = new Pose2d(0, 0, 0);
        drive = new TankDrive(hardwareMap, initialPose);

        // Sync TankDrive params with DriveConstants
        TankDrive.PARAMS.inPerTick = DriveConstants.IN_PER_TICK;
        TankDrive.PARAMS.trackWidthTicks = DriveConstants.TRACK_WIDTH_METERS / DriveConstants.IN_PER_TICK;
        TankDrive.PARAMS.kS = DriveConstants.KS;
        TankDrive.PARAMS.kV = DriveConstants.KV / DriveConstants.IN_PER_TICK;  // Adjust for ticks if needed
        TankDrive.PARAMS.kA = DriveConstants.KA / (DriveConstants.IN_PER_TICK * DriveConstants.IN_PER_TICK);
        TankDrive.PARAMS.maxWheelVel = DriveConstants.metersPerSecToTicksPerSec(DriveConstants.MAX_WHEEL_VEL_METERS_PER_SEC);
        TankDrive.PARAMS.ramseteZeta = DriveConstants.RAMSETE_ZETA;
        TankDrive.PARAMS.ramseteBBar = DriveConstants.RAMSETE_B;

        // Initialize RoadRunnerHelper
        rrHelper = new RoadRunnerHelper(
                hardwareMap, "motorEsquerdo", "motorDireito", driveParams,
                sharedImu, sharedLimelight, initialPose
        );

        // Initialize LimelightDriveController
        llController = new LimelightDriveController(
                hardwareMap, "limelight", "imu",
                hardwareMap.get(DcMotor.class, "motorEsquerdo"),
                hardwareMap.get(DcMotor.class, "motorDireito"),
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                new PIDConfig(0.05, 0, 0),  // distance PID
                new PIDConfig(0.03, 0, 0),  // angle PID
                0.5  // max power
        );

        // Load waypoints from PathPlanner JSON
        List<Pose2d> waypoints = loadPosesFromPathPlannerJson(hardwareMap.appContext, "ftc_path");
        if (waypoints == null || waypoints.isEmpty()) {
            telemetry.addLine("ERROR: Failed to load path JSON!");
            telemetry.update();
            return;
        }

        telemetry.addLine("Path loaded: " + waypoints.size() + " waypoints");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double dt = 0.05;  // Loop interval ~50ms

        // 1. Initial alignment with Limelight to waypoint[0]
        Pose2d firstWaypoint = waypoints.get(0);
        telemetry.addLine("Aligning to initial waypoint: (" + firstWaypoint.position.x + ", " + firstWaypoint.position.y + ")");
        telemetry.update();

        boolean aligned = false;
        while (opModeIsActive() && !aligned) {
            rrHelper.update(dt);
            aligned = llController.moveToPosition(firstWaypoint.position.x, firstWaypoint.position.y, dt, telemetry);
            idle();
        }
        llController.stop();

        // Sync pose after alignment
        rrHelper.setPose(firstWaypoint);
        drive.localizer.setPose(firstWaypoint);
        drive.updatePoseEstimate();
        telemetry.addLine("Alignment complete! Pose: " + firstWaypoint);
        telemetry.update();

        // 2. Follow path with RoadRunner
        if (waypoints.size() > 1) {
            telemetry.addLine("Starting path follow...");
            telemetry.update();

            Pose2d currentPose = firstWaypoint;
            for (int i = 1; i < waypoints.size() && opModeIsActive(); i++) {
                Pose2d targetPose = waypoints.get(i);
                telemetry.addLine("Moving to waypoint " + i + ": " + targetPose);
                telemetry.update();

                // Build spline segment to target
                Action segAction = drive.actionBuilder(currentPose)
                        .splineTo(new Vector2d(targetPose.position.x, targetPose.position.y), targetPose.heading)
                        .build();

                // Run action manually (no dashboard packet)
                while (opModeIsActive() && segAction.run(null)) {
                    rrHelper.update(dt);
                    drive.updatePoseEstimate();
                    telemetry.addData("Current Pose", drive.localizer.getPose().toString());
                    telemetry.addData("Target Pose", targetPose.toString());
                    telemetry.update();
                    idle();
                }

                // Sync pose with fusion after segment
                rrHelper.update(dt);
                drive.localizer.setPose(rrHelper.getPose());
                drive.updatePoseEstimate();
                currentPose = rrHelper.getPose();
                sleep(100);  // Brief pause
            }
        } else {
            telemetry.addLine("Only initial waypoint - path complete.");
        }

        // Stop robot
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        llController.stop();
        rrHelper.update(0);

        telemetry.addLine("Autonomous Finished!");
        telemetry.update();
    }

    /**
     * Loads waypoints from PathPlanner JSON.
     * Extracts x, y, heading (converts degrees to radians) from "waypoints".
     * Ignores extra fields (control lengths, velocity, etc.).
     */
    private List<Pose2d> loadPosesFromPathPlannerJson(android.content.Context context, String fileName) {
        List<Pose2d> poses = new ArrayList<>();
        try {
            InputStream is = context.getAssets().open("paths/" + fileName + ".json");
            BufferedReader br = new BufferedReader(new InputStreamReader(is));
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                sb.append(line);
            }
            br.close();

            JSONObject json = new JSONObject(sb.toString());
            JSONArray waypointsArray = json.getJSONArray("waypoints");

            for (int i = 0; i < waypointsArray.length(); i++) {
                JSONObject wp = waypointsArray.getJSONObject(i);
                double x = wp.getDouble("x");
                double y = wp.getDouble("y");
                double headingDeg = wp.getDouble("heading");
                double headingRad = Math.toRadians(headingDeg);
                poses.add(new Pose2d(x, y, headingRad));
            }
        } catch (Exception e) {
            telemetry.addLine("JSON parse error: " + e.getMessage());
            e.printStackTrace();
            return null;
        }
        return poses;
    }
}
