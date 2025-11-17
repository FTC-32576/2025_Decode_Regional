package ftc.team.Java_Is_AllMight.Utils;


import ftc.team.Java_Is_AllMight.Config.PIDConfig;

/**
 * Unidades: CM, graus (heading)
 */
public class DriveConstants {

    // physical
    public static final double TRACK_WIDTH_CM = 35.56; // 14 in -> cm
    public static final double WHEEL_DIAMETER_CM = 10.16; // 4 in -> cm
    public static final double TICKS_PER_REV = 537.7;

    // conversion
    public static final double CM_PER_TICK = (Math.PI * WHEEL_DIAMETER_CM) / TICKS_PER_REV;

    // control
    public static double MAX_POWER = 0.85;
    public static double MAX_LINEAR_VEL_CM_S = 100.0;
    public static double MAX_ANGULAR_DEG_S = 180.0;

    // tolerances
    public static double POSITION_TOLERANCE_CM = 1.5;
    public static double HEADING_TOLERANCE_DEG = 2.0;

    // ramp
    public static double RAMP_RATE = 0.06;

    // default PID configs (tweak on robot)
    public static double KP_LINEAR = 0.035;
    public static double KI_LINEAR = 0.0;
    public static double KD_LINEAR = 0.002;

    public static double KP_HEADING = 0.012;
    public static double KI_HEADING = 0.0;
    public static double KD_HEADING = 0.001;

        public static final String LEFT_MOTOR = "leftDrive";
        public static final String RIGHT_MOTOR = "rightDrive";

        public static final double MAX_VEL = 0.7;
        public static final double MAX_ACC = 0.03;
        public static final double DECEL_DIST = 25; // cm

        public static final double DESIRED_DIST = 28; // cm da Tag

        public static final PIDConfig FWD_PID = new PIDConfig(0.03, 0, 0.002);
        public static final PIDConfig TURN_PID = new PIDConfig(0.02, 0, 0.001);

        public static final PIDConfig ALIGN_X_PID = new PIDConfig(0.025, 0, 0.001);
        public static final PIDConfig ALIGN_Y_PID = new PIDConfig(0.02, 0, 0.001);
        public static final PIDConfig ALIGN_YAW_PID = new PIDConfig(0.03, 0, 0.001);
        public static final PIDConfig ALIGN_DIST_PID = new PIDConfig(0.022, 0, 0.001);
}
