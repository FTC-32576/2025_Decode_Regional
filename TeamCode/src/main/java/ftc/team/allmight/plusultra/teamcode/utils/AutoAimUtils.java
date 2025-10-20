package ftc.team.allmight.plusultra.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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

    // Ganhos do filtro de Kalman (ajuste fino depois de testar)
    private static final double Q = 0.02;  // variância do processo (confiança na odometria)
    private static final double R = 0.1;   // variância da medição (confiança na Limelight)

    // Estado interno do Kalman
    private static double estimatedX = 0;
    private static double estimatedY = 0;
    private static double estimatedHeading = 0;
    private static double pX = 1, pY = 1, pH = 1;

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

        if (drive != null)
            atualizarPoseComKalman(lime, drive);

        // Se não viu Limelight, fallback usando odometria
        if(drive != null) {
            Pose2d pose = drive.getPoseEstimate();
            Pose2d goalPose = FieldUtils.getGoalPose(alliance);
            return aimWithPose(leftMotor, rightMotor, shooterCommand, pidConfig, maxPower, pose, goalPose);
        }

        return false;
    }

    /** Combina odometria e visão via Filtro de Kalman */
    /**
     * Combina odometria (RoadRunner) e visão (Limelight) com Filtro de Kalman.
     */
    private static void atualizarPoseComKalman(LimeMight lime, SampleTankDrive drive) {
        Pose2d odomPose = drive.getPoseEstimate();

        // Predição com base na odometria
        double xPred = odomPose.getX();
        double yPred = odomPose.getY();
        double hPred = odomPose.getHeading();

        // Atualiza incerteza
        pX += Q;
        pY += Q;
        pH += Q;

        // Correção com visão (se disponível)
        Pose2d visionPose = null;

        if (lime != null && lime.isTargetValid()) {
            Pose3D pose3d = lime.getBotPoseMT2(); //retorna Pose3D

            if (pose3d != null) {
                double x = pose3d.getPosition().x; // metros
                double y = pose3d.getPosition().y; // metros
                double headingDeg = pose3d.getOrientation().getYaw(AngleUnit.DEGREES); // yaw
                visionPose = new Pose2d(x * 100, y * 100, Math.toRadians(headingDeg)); // converte para cm
            }
        }

        if (visionPose != null) {
            // Ganhos do Kalman
            double kX = pX / (pX + R);
            double kY = pY / (pY + R);
            double kH = pH / (pH + R);

            // Atualiza estado
            estimatedX = xPred + kX * (visionPose.getX() - xPred);
            estimatedY = yPred + kY * (visionPose.getY() - yPred);
            estimatedHeading = hPred + kH * (visionPose.getHeading() - hPred);

            // Atualiza incerteza
            pX = (1 - kX) * pX;
            pY = (1 - kY) * pY;
            pH = (1 - kH) * pH;
        } else {
            // Sem visão → mantém predição
            estimatedX = xPred;
            estimatedY = yPred;
            estimatedHeading = hPred;
        }

        // Atualiza pose do drive
        drive.setPoseEstimate(new Pose2d(estimatedX, estimatedY, estimatedHeading));
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
