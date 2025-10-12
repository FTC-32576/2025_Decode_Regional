package ftc.team.allmight.plusultra.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.Java_Is_AllMight.Config.PIDConfig;

public class ShooterSubsystem {

    private final DcMotorEx shooterMotor;
    private final PIDController shooterPID;
    private final Telemetry telemetry;

    // Configuração PID (ajustar conforme seu motor)
    public static PIDConfig pidConfig = new PIDConfig(0.005, 0, 0, 0.00021);
    public static double TICKS_PER_REV = 28; // ticks por rotação do motor

    public double shooterTarget = 0;
    private double offset = 0; // ajuste fino

    public ShooterSubsystem(HardwareMap hardwareMap, String shooterName, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooterMotor = hardwareMap.get(DcMotorEx.class, shooterName);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooterPID = new PIDController(pidConfig);
    }

    // Liga o shooter a potência fixa (teleop simples)
    public void spin(double power) {
        shooterMotor.setPower(power);
        shooterTarget = 0; // desativa PID
        shooterPID.reset();
    }

    // Liga o shooter para atingir RPM desejado
    public void spinRPM(double rpm) {
        shooterTarget = rpm;
    }

    // Para o shooter
    public void stop() {
        shooterTarget = 0;
        shooterMotor.setPower(0);
        shooterPID.reset();
    }

    // Ajuste fino do RPM durante a partida
    public void adjustShooterRPM(double adjustment) {
        offset += adjustment;
    }

    // Atualizar PID e aplicar potência — chamar a cada ciclo
    public void update() {
        if (shooterTarget > 0) {
            double currentRPM = getShooterRPM();
            double power = shooterPID.calculate(shooterTarget + offset, currentRPM);
            shooterMotor.setPower(power);
        }

        // Telemetria básica para debug
        telemetry.addData("Shooter RPM", getShooterRPM());
        telemetry.addData("Shooter Setpoint", shooterTarget + offset);
        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.update();
    }

    // Retorna RPM atual do motor
    public double getShooterRPM() {
        return 60.0 * (shooterMotor.getVelocity() / TICKS_PER_REV);
    }

    // Retorna se o shooter está dentro da tolerância
    public boolean atSetpoint() {
        return Math.abs(shooterTarget + offset - getShooterRPM()) < 50; // ±50 RPM
    }

    // Permite inverter direção do motor se necessário
    public void setDirection(DcMotorSimple.Direction direction) {
        shooterMotor.setDirection(direction);
    }
}
