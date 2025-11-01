package ftc.team.allmight.plusultra.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.allmight.plusultra.teamcode.utils.MathUtils;

public class ShooterSubsystem {

    // --- Hardware ---
    private final DcMotorEx shooterMotor;

    // --- Controle PIDF ---
    private final PIDController velocityController;
    private final PIDConfig pidConfig;

    // --- Feedforward físico ---
    private final double kV;  // proporcional à velocidade (principal termo do FF)
    private final double kS;  // voltagem estática (vence atrito)
    private final double kA;  // aceleração → voltagem (pouco usado em shooter)

    // --- Variáveis de controle ---
    private double targetVelocityTicks = 0; // ticks/s internamente
    private double lastVelocity = 0;
    private double lastRPM = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private static final double TICKS_PER_REV = 28.0;

    // --- Configuração base ---
    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        pidConfig = new PIDConfig(
                0.0008,  // kP
                0.00002, // kI
                0.0001,  // kD
                0.00025, // kF
                0
        );

        velocityController = new PIDController(pidConfig);

        kV = 0.00025;
        kS = 0.05;
        kA = 0.0;

        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        timer.reset();
    }

    // --- Atualiza o controle do flywheel (loop) ---
    public void update() {
        double currentVelocity = shooterMotor.getVelocity(); // ticks/s
        double dt = timer.seconds();
        timer.reset();

        double acceleration = (currentVelocity - lastVelocity) / Math.max(dt, 0.001);
        lastVelocity = currentVelocity;

        // Feedforward físico
        double physicalFF = (kS * Math.signum(targetVelocityTicks))
                + (kV * targetVelocityTicks)
                + (kA * acceleration);

        // PIDF (PID + feedforward)
        double pidOutput = velocityController.calculate(targetVelocityTicks, currentVelocity);

        double totalOutput = pidOutput + physicalFF;
        totalOutput = Math.max(-1.0, Math.min(totalOutput, 1.0));

        shooterMotor.setPower(totalOutput);
    }

    // --- Define velocidade alvo em RPM ---
    public void setTargetVelocity(double targetRPM) {
        // converte RPM para ticks/s
        this.targetVelocityTicks = (targetRPM / 60.0) * TICKS_PER_REV;
    }

    // --- Ajuste dinâmico baseado em distância até o goal ---
    public void setTargetVelocityByDistance(Pose2d robotPose, double maxRPM) {
        double dX = robotPose.getX();
        double dY = robotPose.getY();
        double distancia = MathUtils.calcularDistanciaAteOGoal(dY, dX);
        double powerFactor = MathUtils.CalculateShooterPower(distancia); // 0.4 → 1.0
        setTargetVelocity(powerFactor * maxRPM);
    }

    // --- Função de status: pronto pra disparar? ---
    public boolean isReadyToShoot() {
        double currentRPM = ticksToRPM(shooterMotor.getVelocity());
        double targetRPM = ticksToRPM(targetVelocityTicks);

        double error = Math.abs(targetRPM - currentRPM);
        boolean withinError = error < 50; // tolerância ±50 RPM

        boolean stable = Math.abs(currentRPM - lastRPM) < 30;
        lastRPM = currentRPM;

        return withinError && stable;
    }

    // --- Conversão ticks/s <-> RPM ---
    private double ticksToRPM(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    public double getCurrentRPM() {
        return ticksToRPM(shooterMotor.getVelocity());
    }

    public double getPowerOutput() {
        return shooterMotor.getPower();
    }

    public void stop() {
        targetVelocityTicks = 0;
        shooterMotor.setPower(0);
        velocityController.reset();
    }
}
