package ftc.team.Java_Is_AllMight.Config;

public class DriveConstants {
    public static final double TRACK_WIDTH = 33.0;
    public static final double WHEEL_RADIUS = 5.0;
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_VEL = 60.0;
    public static final double MAX_ACCEL = 45.0;
    public static final double MAX_ANG_VEL = Math.toRadians(180);
    public static final double MAX_ANG_ACCEL = Math.toRadians(180);

    public static double encoderTicksToCm(double ticks) {
        return (ticks / TICKS_PER_REV) * (2 * Math.PI * WHEEL_RADIUS);
    }
}