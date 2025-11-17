package ftc.team.Java_Is_AllMight.PlusPathing;


import java.util.ArrayList;
import java.util.List;

public class PlusTrajectorySequence {

    private final List<TrajectoryCommand> commands;

    public PlusTrajectorySequence(List<TrajectoryCommand> commands){
        this.commands = commands;
    }

    public List<TrajectoryCommand> getCommands(){
        return commands;
    }
}
