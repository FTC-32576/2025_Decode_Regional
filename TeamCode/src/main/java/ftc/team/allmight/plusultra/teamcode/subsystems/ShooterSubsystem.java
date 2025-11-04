package ftc.team.allmight.plusultra.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.Java_Is_AllMight.Utils.RoboUtils;
import ftc.team.allmight.plusultra.teamcode.utils.MathUtils;

public class ShooterSubsystem {

    // ---------- HARDWARE ----------
    private DcMotorEx shooterMotor;

    // ---------- CONTROLE PID ----------
    private PIDController pidController;
    private PIDConfig pidConfig;

    // ---------- VARIÁVEIS ----------
    private double targetVelocityTicks = 0;
    private double lastRPM = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private static final double TICKS_PER_REV = 28.0;
    private static final double DEFAULT_RPM = 6000.0;

    private static RoboUtils roboUtils = new RoboUtils();

    // ---------- INICIALIZAÇÃO ----------
    public void init(HardwareMap hardwareMap) {
        shooterMotor = roboUtils.getHardware(hardwareMap, DcMotorEx.class, "intake");

        pidConfig = new PIDConfig(
                0.0008,  // kP
                0.00002, // kI
                0.0001,  // kD
                0.00025  // kF
        );

        pidController = new PIDController(pidConfig);

        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer.reset();
    }

    // ---------- LOOP PRINCIPAL ----------
    public void update(Telemetry telemetry) {
        double currentVelocity = shooterMotor.getVelocity();
        double pidOutput = pidController.calculate(targetVelocityTicks, currentVelocity);
        pidOutput = Math.max(-1.0, Math.min(pidOutput, 1.0));
//        shooterMotor.setPower(pidOutput);tui[´~çplokj

        telemetry.addData("Shooter Power (PID)", pidOutput);
    }

    public void updateShooterFromCamera(double distanceMeters, Telemetry telemetry) {
        double power = MathUtils.calculateShooterPower(distanceMeters);

        // shooterMotor.setPower(power); // desligado até ter o motor
        telemetry.addData("Shooter Target Power", String.format("%.2f", power));
        telemetry.addData("Distance (m)", String.format("%.2f", distanceMeters));
    }

    public void shoot() {
        setTargetVelocity(DEFAULT_RPM);
    }


    private void setTargetVelocity(double targetRPM) {
        targetVelocityTicks = (targetRPM / 60.0) * TICKS_PER_REV;
    }

    public boolean isReadyToShoot() {
        double currentRPM = ticksToRPM(shooterMotor.getVelocity());
        double targetRPM = ticksToRPM(targetVelocityTicks);
        double error = Math.abs(targetRPM - currentRPM);
        boolean withinError = error < 50;
        boolean stable = Math.abs(currentRPM - lastRPM) < 30;
        lastRPM = currentRPM;
        return withinError && stable;
    }

    private double ticksToRPM(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    public double getCurrentRPM() {
        return ticksToRPM(shooterMotor.getVelocity());
    }

    public double getPowerOutput() {
        return shooterMotor.getPower();
    }

    // ---------- PARAR O SHOOTER ----------
    public void stop() {
        targetVelocityTicks = 0;
        shooterMotor.setPower(0);
        pidController.reset();
    }
}
