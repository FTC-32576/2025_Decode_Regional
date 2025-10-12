package ftc.team.allmight.plusultra.teamcode.commands;

import ftc.team.allmight.plusultra.teamcode.subsystems.ShooterSubsystem;

public class ShooterCommand {

    private final ShooterSubsystem shooter;

    public ShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    // Liga shooter a potência fixa (modo teleop simples)
    public void execute(double power) {
        shooter.spin(power);
    }

    // Liga shooter para atingir RPM específico usando PID
    public void executeRPM(double rpm) {
        shooter.spinRPM(rpm);
    }

    // Chamar a cada ciclo do teleop para atualizar PID
    public void update() {
        shooter.update();
    }

    // Para o shooter e reseta PID
    public void end() {
        shooter.stop();
    }

    // Checa se o shooter atingiu o setpoint
    public boolean atSetpoint() {
        return shooter.atSetpoint();
    }

    // Ajuste fino do RPM durante a partida
    public void adjustRPM(double adjustment) {
        shooter.adjustShooterRPM(adjustment);
    }
}
