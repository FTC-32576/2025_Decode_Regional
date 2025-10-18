package ftc.team.allmight.plusultra.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;

import ftc.team.allmight.plusultra.teamcode.commands.ShooterCommand;
import ftc.team.allmight.plusultra.teamcode.roadrunner.drive.SampleTankDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.Java_Is_AllMight.Config.PIDConfig;

public class AutoAimUtils {

    /**
     * Gira o robô usando PID até mirar no alvo e ajusta shooter.
     * @param drive      Instância do SampleTankDrive do RoadRunner
     * @param leftMotor  Motor esquerdo
     * @param rightMotor Motor direito
     * @param shooter    Comando do shooter
     * @param goalPose   Posição do GOAL
     * @param pidConfig  Configuração do PID para o giro
     */
    public static void aimAndAdjustPID(SampleTankDrive drive,
                                       DcMotor leftMotor,
                                       DcMotor rightMotor,
                                       ShooterCommand shooter,
                                       Pose2d goalPose,
                                       PIDConfig pidConfig) {

        PIDController turnPID = new PIDController(pidConfig);

        // Atualiza odometria
        drive.update();
        Pose2d pose = drive.getPoseEstimate();

        // Calcula posição relativa
        double relX = goalPose.getX() - pose.getX();
        double relY = goalPose.getY() - pose.getY();
        Pose2d relativePose = new Pose2d(relX, relY, pose.getHeading());

        // Calcula mira
        ResultadoMira resultado = MathUtils.calcularMira(relativePose);

        // Ajusta potência do shooter
        double shooterPower = MathUtils.CalculateShooterPower(resultado.distancia);
        shooter.execute(shooterPower);

        // Calcula erro angular
        double targetAngleDeg = Math.toDegrees(resultado.angulo);
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());
        double angleError = targetAngleDeg - robotHeadingDeg;

        // Normaliza para -180..180
        while (angleError > 180) angleError -= 360;
        while (angleError < -180) angleError += 360;

        // Calcula potência do giro via PID
        double turnPower = Range.clip(turnPID.calculate(0, -angleError), -0.5, 0.5);

        // Aplica potência aos motores
        leftMotor.setPower(-turnPower);
        rightMotor.setPower(turnPower);
    }
}
