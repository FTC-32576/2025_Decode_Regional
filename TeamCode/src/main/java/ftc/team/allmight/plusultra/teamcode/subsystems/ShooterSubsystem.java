package ftc.team.allmight.plusultra.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.allmight.plusultra.teamcode.utils.MathUtils;

public class ShooterSubsystem {

    // ===== Estados =====
    public enum ShooterState {
        OFF,
        SPINUP_RPM,
        SPINUP_POWER,
        READY,
        HOLD
    }

    private ShooterState currentState = ShooterState.OFF;

    // ===== Hardware =====
    private DcMotorEx shooterMotor;

    // ===== PID =====
    private PIDController pid;
    private PIDConfig pidConfig;

    // ===== Variáveis =====
    private double targetVelocityTicks = 0;
    private double targetShooterRPM = 0; // RPM real do flywheel
    private double manualPower = 0;
    private double lastShooterRPM = 0;

    private final ElapsedTime spinupTimer = new ElapsedTime();

    // ===== Constantes =====
    private static final double TICKS_PER_REV = 28.0; // encoder do motor REV
    private static final double GEAR_RATIO = 10.0 / 25.0;

    private static final double READY_ERROR_RPM = 40;  // margem de erro aceitável
    private static final double STABLE_DELTA_RPM = 25; // variação máxima para ser considerado estável


    // INIT

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        pidConfig = new PIDConfig(
                0.0008,
                0.00002,
                0.0001,
                0.00025
        );

        pid = new PIDController(pidConfig);

        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    // UPDATE

    public void update(Telemetry telemetry) {
        double ticksPerSecond = shooterMotor.getVelocity();
        double shooterRPM = ticksToShooterRPM(ticksPerSecond);

        switch (currentState) {

            case OFF: shooterMotor.setPower(0); pid.reset(); break;

            case SPINUP_POWER:
                shooterMotor.setPower(manualPower); if (isReady(shooterRPM)) {currentState = ShooterState.READY;}
                break;

            case READY: double hold = pid.calculate(targetVelocityTicks, ticksPerSecond);
            shooterMotor.setPower(clamp(hold, 0, 1));break;
        }

        telemetry.addData("Shooter State", currentState);
        telemetry.addData("Shooter RPM (real)", shooterRPM);
        telemetry.addData("Target RPM", targetShooterRPM);
        telemetry.addData("Spin-up (s)", getSpinupTimeSeconds());

        lastShooterRPM = shooterRPM;
    }

    // MÉTODOS DE CONTROLE DO SHOOTER

    public void setRPM(double shooterRPM) {
        this.targetShooterRPM = shooterRPM;

        // Converter RPM real do flywheel para RPM do motor
        double motorRPM = shooterRPM / GEAR_RATIO;

        // Converter motor RPM para ticks/s
        targetVelocityTicks = rpmToTicks(motorRPM);

        spinupTimer.reset();
        currentState = ShooterState.SPINUP_RPM;
    }

    public void setPower(double power) {
        manualPower = clamp(power, 0, 1);
        spinupTimer.reset();
        currentState = ShooterState.SPINUP_POWER;
    }

    public void shootFromDistance(double meters) {
        double power = MathUtils.calculateShooterPower(meters);
        setPower(power);
    }

    public void stop() {
        currentState = ShooterState.OFF;
    }

    public boolean isReadyToShoot() {
        return currentState == ShooterState.READY || currentState == ShooterState.HOLD;
    }


    // FUNÇÕES INTERNAS

    private boolean isReady(double shooterRPM) {
        double error = Math.abs(targetShooterRPM - shooterRPM);
        boolean withinError = error < READY_ERROR_RPM;
        boolean stable = Math.abs(shooterRPM - lastShooterRPM) < STABLE_DELTA_RPM;
        return withinError && stable;
    }

    private double rpmToTicks(double motorRPM) {
        return (motorRPM / 60.0) * TICKS_PER_REV;  // motor rev/s → ticks/s
    }

    private double ticksToShooterRPM(double ticksPerSecond) {
        double motorRPM = (ticksPerSecond / TICKS_PER_REV) * 60.0;
        return motorRPM * GEAR_RATIO; // aplicar redução
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }

    public double getRPM() {
        return ticksToShooterRPM(shooterMotor.getVelocity());
    }

    public double getSpinupTimeSeconds() {
        return spinupTimer.seconds();
    }
}
