package ftc.team.allmight.plusultra.teamcode.commands;

import ftc.team.allmight.plusultra.teamcode.subsystems.ShooterSubsystem;

public class ShooterCommand {

    private final ShooterSubsystem shooter;


    public ShooterCommand(ShooterSubsystem shooter){
        this.shooter = shooter;

    }

    public void execute (double power){
        shooter.spin(power);
    }

    public void end(){
        shooter.stop();
    }

}
