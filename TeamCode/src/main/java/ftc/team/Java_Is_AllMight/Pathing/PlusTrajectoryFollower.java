package ftc.team.Java_Is_AllMight.Pathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import ftc.team.Java_Is_AllMight.Pathing.PlusPathing;

public class PlusTrajectoryFollower {

    private final PlusPathing pathing;
    private PlusTrajectory trajectory;
    private int index = 0;

    public PlusTrajectoryFollower(PlusPathing pathing) {
        this.pathing = pathing;
    }

    public void follow(PlusTrajectory trajectory) {
        this.trajectory = trajectory;
        this.index = 0;
    }

    public PlusPathing.ControlOutput update(Pose2d pose) {

        if (trajectory == null)
            return new PlusPathing.ControlOutput(0,0,0);

        if (index >= trajectory.getSegments().size())
            return new PlusPathing.ControlOutput(0,0,0);

        PlusTrajectory.Segment target = trajectory.getSegments().get(index);
        PlusPathing.Waypoint wp = new PlusPathing.Waypoint(target.x, target.y, target.heading);

        PlusPathing.ControlOutput out = pathing.update(pose, wp);

        // Quando chegar perto, avança para o próximo
        if (out.remaining < 3.0) {
            index++;
        }

        return out;
    }

    public boolean isFinished() {
        return trajectory != null &&
                index >= trajectory.getSegments().size();
    }
}
