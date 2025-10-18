package ftc.team.allmight.plusultra.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

import ftc.team.Java_Is_AllMight.Sensors.LimeMight;
import ftc.team.Java_Is_AllMight.Utils.Alliance;
import ftc.team.allmight.plusultra.teamcode.commands.ShooterCommand;
import ftc.team.allmight.plusultra.teamcode.roadrunner.drive.SampleTankDrive;
import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.Java_Is_AllMight.Config.PIDConfig;

public class AutoAimUtils {

    private static final double TARGET_DISTANCE = 1000; // mm
    private static final double DISTANCE_TOLERANCE = 50; // mm
    private static final double YAW_TOLERANCE = 2;       // °

    // Métodoo: tenta mirar usando Limelight e odometria
    public static boolean aimAjust(LimeMight lime,
                                   Alliance alliance,
                                   SampleTankDrive drive,
                                   DcMotor leftMotor,
                                   DcMotor rightMotor,
                                   ShooterCommand shooterCommand,
                                   PIDConfig pidConfig,
                                   double maxPower) {

        if(leftMotor == null || rightMotor == null || shooterCommand == null || alliance == null)
            return false;

        //  Tenta usar Limelight, se tiver
        FiducialResult fid = getTargetFiducial(lime, alliance);
        if(fid != null) {
            return aimWithLimelight(lime, fid, leftMotor, rightMotor, shooterCommand, pidConfig, maxPower);
        }

        // Se não viu Limelight, fallback usando odometria
        if(drive != null) {
            Pose2d pose = drive.getPoseEstimate();
            Pose2d goalPose = FieldUtils.getGoalPose(alliance);
            return aimWithPose(leftMotor, rightMotor, shooterCommand, pidConfig, maxPower, pose, goalPose);
        }

        return false;
    }

    /** Pega a tag da aliança se o Limelight estiver ativo */
    private static FiducialResult getTargetFiducial(LimeMight lime, Alliance alliance) {
        if(lime == null) return null;
        int targetID = alliance.getTagID();

        List<FiducialResult> fiducials = lime.getFiducials();
        if(fiducials != null) {
            for(FiducialResult f : fiducials) {
                if(f.getFiducialId() == targetID) return f;
            }
        }
        return null;
    }

    /** Mira usando Limelight */
    private static boolean aimWithLimelight(LimeMight lime,
                                            FiducialResult fid,
                                            DcMotor leftMotor,
                                            DcMotor rightMotor,
                                            ShooterCommand shooterCommand,
                                            PIDConfig pidConfig,
                                            double maxPower) {
        PIDController turnPID = new PIDController(pidConfig);

        double yaw = lime.getYawToFiducial(fid);
        double distance = lime.getDistanceToFiducial(fid);
        double turnPower = Range.clip(turnPID.calculate(0, yaw), -maxPower, maxPower);

        leftMotor.setPower(-turnPower);
        rightMotor.setPower(turnPower);

        boolean aligned = Math.abs(yaw) < YAW_TOLERANCE && Math.abs(distance - TARGET_DISTANCE) < DISTANCE_TOLERANCE;

        if(aligned) shooterCommand.execute(MathUtils.CalculateShooterPower(distance / 10));
        else shooterCommand.end();

        return aligned;
    }

    /** Mira usando apenas a pose estimada (odometria / RoadRunner) */
    private static boolean aimWithPose(DcMotor leftMotor,
                                       DcMotor rightMotor,
                                       ShooterCommand shooterCommand,
                                       PIDConfig pidConfig,
                                       double maxPower,
                                       Pose2d currentPose,
                                       Pose2d goalPose) {

        PIDController turnPID = new PIDController(pidConfig);

        double relX = goalPose.getX() - currentPose.getX();
        double relY = goalPose.getY() - currentPose.getY();

        double yaw = Math.toDegrees(Math.atan2(relY, relX)) - Math.toDegrees(currentPose.getHeading());
        while(yaw > 180) yaw -= 360;
        while(yaw < -180) yaw += 360;

        double distance = Math.hypot(relX, relY);
        double turnPower = Range.clip(turnPID.calculate(0, yaw), -maxPower, maxPower);

        leftMotor.setPower(-turnPower);
        rightMotor.setPower(turnPower);

        boolean aligned = Math.abs(yaw) < YAW_TOLERANCE && Math.abs(distance - TARGET_DISTANCE) < DISTANCE_TOLERANCE;

        if(aligned) shooterCommand.execute(MathUtils.CalculateShooterPower(distance / 10));
        else shooterCommand.end();

        return aligned;
    }
}
