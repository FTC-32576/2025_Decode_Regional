package ftc.team.Java_Is_AllMight.Pathing;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;

/**
 * Trajectory-style API for PlusPathing.
 * Usage:
 * new PlusTrajectoryBuilder(startPose)
 *     .toWaypoint(50, 60, Math.toRadians(90))
 *     .toWaypoint(80, 90)
 *     .build();
 */
public class PlusTrajectory {

    public static class Segment {
        public final double x, y, heading;
        public Segment(double x, double y, double heading){
            this.x=x; this.y=y; this.heading=heading;
        }
    }

    private final List<Segment> segments;
    public PlusTrajectory(List<Segment> segments){
        this.segments = segments;
    }

    public List<Segment> getSegments(){ return segments; }

}
