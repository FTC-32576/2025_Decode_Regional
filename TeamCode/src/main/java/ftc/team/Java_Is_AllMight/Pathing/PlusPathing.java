package ftc.team.Java_Is_AllMight.Pathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;
import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Config.PIDController;

public class PlusPathing {

    public static class Waypoint {
        public double x, y, heading;
        public Waypoint(double x, double y, double heading){
            this.x=x; this.y=y; this.heading=heading;
        }
    }

    private final List<Waypoint> path = new ArrayList<>();
    private final PIDController lateralPID;
    private final PIDController headingPID;

    public PlusPathing(PIDConfig lateralConfig, PIDConfig headingConfig) {
        this.lateralPID = new PIDController(lateralConfig);
        this.headingPID = new PIDController(headingConfig);
    }

    public PlusPathing addWaypoint(double x, double y, double heading){
        path.add(new Waypoint(x, y, heading));
        return this;
    }

    public List<Waypoint> build(){
        return path;
    }

    public ControlOutput update(Pose2d pose, Waypoint target){
        double dx = target.x - pose.getX();
        double dy = target.y - pose.getY();
        double targetHeading = target.heading;

        double distance = Math.hypot(dx, dy);
        double angleToPoint = Math.atan2(dy, dx);

        double lateralError = distance;
        double headingError = angleToPoint - pose.getHeading();

        while(headingError > Math.PI) headingError -= 2*Math.PI;
        while(headingError < -Math.PI) headingError += 2*Math.PI;

        double drive = lateralPID.calculate(0, -lateralError);
        double turn = headingPID.calculate(0, -headingError);

        return new ControlOutput(drive, turn, distance);
    }

    public static class ControlOutput {
        public double drive, turn, remaining;
        public ControlOutput(double drive, double turn, double remaining){
            this.drive=drive;
            this.turn=turn;
            this.remaining=remaining;
        }
    }

}


