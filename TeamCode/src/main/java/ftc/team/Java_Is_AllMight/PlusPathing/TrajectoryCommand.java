package ftc.team.Java_Is_AllMight.PlusPathing;
public class TrajectoryCommand {

    public enum Type {
        FORWARD,
        BACKWARD,
        TURN
    }

    public final Type type;
    public final double value;

    public TrajectoryCommand(Type type, double value) {
        this.type = type;
        this.value = value;
    }
}
