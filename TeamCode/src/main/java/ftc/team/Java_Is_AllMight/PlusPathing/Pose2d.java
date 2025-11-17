package ftc.team.Java_Is_AllMight.PlusPathing;

public class Pose2d {
    public double x;       // cm
    public double y;       // cm
    public double heading; // deg

    public Pose2d() { this(0,0,0); }
    public Pose2d(double x, double y, double heading) {
        this.x = x; this.y = y; this.heading = heading;
    }

    public Pose2d copy(){ return new Pose2d(x,y,heading); }

    @Override
    public String toString() {
        return String.format("Pose2d{%.1fcm, %.1fcm, %.1f°}", x, y, heading);
    }
}
