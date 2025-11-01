package ftc.team.Java_Is_AllMight.Pathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;

public class PlusTrajectoryBuilder {

    private final List<PlusTrajectory.Segment> segments = new ArrayList<>();
    private Pose2d currentPose;

    public PlusTrajectoryBuilder(Pose2d startPose) {
        this.currentPose = startPose;
    }

    public PlusTrajectoryBuilder toWaypoint(double x, double y) {
        double heading = currentPose.getHeading();
        segments.add(new PlusTrajectory.Segment(x, y, heading));
        currentPose = new Pose2d(x, y, heading);
        return this;
    }

    public PlusTrajectoryBuilder toWaypoint(double x, double y, double heading) {
        segments.add(new PlusTrajectory.Segment(x, y, heading));
        currentPose = new Pose2d(x, y, heading);
        return this;
    }

    public PlusTrajectory build() {
        return new PlusTrajectory(new ArrayList<>(segments));
    }
}
