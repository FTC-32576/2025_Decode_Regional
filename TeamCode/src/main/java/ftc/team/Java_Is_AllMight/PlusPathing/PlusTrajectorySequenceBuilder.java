package ftc.team.Java_Is_AllMight.PlusPathing;

import java.util.ArrayList;
import java.util.List;

public class PlusTrajectorySequenceBuilder {

    private final List<TrajectoryCommand> commands = new ArrayList<>();

    public PlusTrajectorySequenceBuilder forward(double cm){
        commands.add(new TrajectoryCommand(TrajectoryCommand.Type.FORWARD, cm));
        return this;
    }

    public PlusTrajectorySequenceBuilder backward(double cm){
        commands.add(new TrajectoryCommand(TrajectoryCommand.Type.BACKWARD, cm));
        return this;
    }

    public PlusTrajectorySequenceBuilder turn(double degrees){
        commands.add(new TrajectoryCommand(TrajectoryCommand.Type.TURN, degrees));
        return this;
    }

    public PlusTrajectorySequence build(){
        return new PlusTrajectorySequence(new ArrayList<>(commands));
    }
}

