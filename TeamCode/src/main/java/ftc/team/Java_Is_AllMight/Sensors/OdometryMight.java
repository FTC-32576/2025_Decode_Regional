package ftc.team.Java_Is_AllMight.Sensors;

public class OdometryMight {

    private final EncoderMight left;
    private final EncoderMight right;
    private final IMUMight imu;

    // Odometry absoluta
    private double x = 0;
    private double y = 0;

    // Para cálculo incremental
    private double lastLeft = 0;
    private double lastRight = 0;

    // Distância total (para movimentos forward/backward)
    private double forwardDistance = 0;

    public OdometryMight(EncoderMight left, EncoderMight right, IMUMight imu) {
        this.left = left;
        this.right = right;
        this.imu = imu;

        lastLeft = left.getDistanceCm();
        lastRight = right.getDistanceCm();
    }

    // ============================================================================
    // UPDATE
    // ============================================================================

    public void update() {

        double leftDist = left.getDistanceCm();
        double rightDist = right.getDistanceCm();

        double dL = leftDist - lastLeft;
        double dR = rightDist - lastRight;

        lastLeft = leftDist;
        lastRight = rightDist;

        double forward = (dL + dR) / 2.0;

        // acumula para forward()
        forwardDistance += forward;

        // Odometry absoluta (x,y)
        double headingRad = Math.toRadians(getHeading());
        x += forward * Math.cos(headingRad);
        y += forward * Math.sin(headingRad);
    }

    // ============================================================================
    // DIST FOR MOTION
    // ============================================================================

    public void resetDistance() {
        forwardDistance = 0;
    }

    public double getForwardDistance() {
        return forwardDistance;
    }

    // ============================================================================
    // HEADING
    // ============================================================================

    public double getHeading() {
        return imu.getYaw();
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    // ============================================================================
    // ABSOLUTE ODOMETRY
    // ============================================================================

    public double getX() { return x; }
    public double getY() { return y; }

    public void resetXY() {
        x = 0;
        y = 0;
    }

    // Reset completo
    public void reset() {
        x = 0;
        y = 0;
        forwardDistance = 0;

        lastLeft = left.getDistanceCm();
        lastRight = right.getDistanceCm();
    }
}
