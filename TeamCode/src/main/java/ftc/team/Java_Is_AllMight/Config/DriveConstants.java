package ftc.team.Java_Is_AllMight.Config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * DriveConstants - Set of constants and drivetrain parameters.
 *
 * This class centralizes configurations for RoadRunner, TankDrive, MecanumDrive, and odometry.
 * Use it for tuning and calibration, ensuring consistency across modules (e.g., RoadRunnerHelper, TankDrive).
 *
 * Values are in SI units (meters, radians, seconds) for RoadRunner compatibility.
 *
 * Automatic Initialization:
 * - Call DriveConstants.initialize(...) once in the OpMode init (e.g., before creating TankDrive or RoadRunnerHelper).
 * - Provide hardware parameters: wheel diameter (inches), gear ratio (e.g., 20.0 for REV HD Hex 20:1),
 *   encoder CPR (e.g., 28 for REV built-in), and track width (inches).
 * - The method automatically calculates IN_PER_TICK, WHEEL_RADIUS_METERS, TICKS_PER_REVOLUTION, etc.
 * - Feedforward, limits, and Ramsete values are predefined (tune manually if needed).
 *
 * Considerations for REV HD Hex Motors:
 * - Built-in encoder: 28 CPR (counts per revolution) on the motor shaft.
 * - Quadrature: Multiplied by 4 for direction/precision (standard FTC/RoadRunner).
 * - Typical gear ratio: 20:1 or 40:1 (output shaft rotates slower).
 * - Example for REV HD Hex 20:1 + 4" wheel (diameter=4.0 inches):
 *   - Ticks per rev = 28 CPR * 20 gear * 4 quadrature = 2240 ticks/output rev.
 *   - Radius = 2" = 0.0508 m; Circumference ≈ 0.319 m.
 *   - IN_PER_TICK ≈ 0.319 / 2240 ≈ 0.000142 m/tick.
 *
 * Tuning Instructions (after init):
 * - Feedforward (KS, KV, KA): Use the RoadRunner System Identification tool.
 * - Limits: Test maximum safe speeds/accelerations (battery/motors).
 * - Ramsete: Zeta for damping (0.5-1.0), B for aggressiveness (1.5-3.0).
 * - Tolerances: Adjust for desired path precision.
 *
 * For reusability: Call initialize() with your hardware params in each OpMode.
 * Integrate with TankDrive: After init, copy values to TankDrive.PARAMS (see example in comments).
 */
public final class DriveConstants {
    private DriveConstants() {}

    // ========================================================================
    // CALCULATED FIELDS (Updated via initialize())
    // ========================================================================
    /**
     * Distance traveled per encoder tick (meters/tick). Calculated automatically.
     */
    public static double IN_PER_TICK;

    /**
     * Ticks per encoder revolution (calculated: encoderCPR * gearRatio * 4 for quadrature).
     */
    public static int TICKS_PER_REVOLUTION;

    /**
     * Wheel radius (meters). Calculated from wheelDiameterInches.
     */
    public static double WHEEL_RADIUS_METERS;

    /**
     * Track width: Distance between wheel centers (meters).
     * Calculated from trackWidthInches.
     */
    public static double TRACK_WIDTH_METERS;

    // ========================================================================
    // HARDWARE PARAMETERS (Predefined or Passed in Init)
    // ========================================================================
    // These are examples; pass real values in initialize() for accurate calculation
    private static double DEFAULT_WHEEL_DIAMETER_INCHES = 3.5;  // Standard 4" wheel (diameter)
    private static double DEFAULT_GEAR_RATIO = 20.0;  // REV HD Hex 20:1
    private static int DEFAULT_ENCODER_CPR = 28;  // REV built-in encoder CPR (motor shaft)
    private static double DEFAULT_TRACK_WIDTH_INCHES = 18.0;  // Standard track width ~18"

    // ========================================================================
    // FEEDFORWARD (Motor Control - Tune Manually After Init)
    // ========================================================================
    /**
     * Static gain (kS): Compensation for static friction (typical 0.0-0.5).
     * Tune with System ID: Value that starts motion without overshoot.
     */
    public static double KS = 0.15;  // Conservative initial value

    /**
     * Velocity gain (kV): Compensation for velocity (typical 1.0-2.0 in volts/(m/s)).
     * Tune: Maintains constant velocity without acceleration.
     */
    public static double KV = 1.0;  // Initial; tune for gearing/battery

    /**
     * Acceleration gain (kA): Compensation for acceleration (typical 0.0-0.1 in volts/(m/s²)).
     * Tune: Reduces error during rapid accelerations.
     */
    public static double KA = 0.02;  // Low initial; increase if oscillating

    // ========================================================================
    // PROFILE LIMITS (Trajectories and Maximum Speed)
    // ========================================================================
    /**
     * Maximum wheel velocity (m/s). Limits forward/strafe.
     * Typical: 2.0-3.0 m/s (based on 12V battery).
     */
    public static double MAX_WHEEL_VEL_METERS_PER_SEC = 2.5;

    /**
     * Maximum profile acceleration (m/s²). Limits linear acceleration.
     * Typical: 2.0-4.0 m/s² for smoothness.
     */
    public static double MAX_ACCEL_METERS_PER_SEC_SQ = 3.0;

    /**
     * Maximum angular velocity (rad/s). For turns and curves.
     * Typical: π rad/s (~180°/s) for tank drive.
     */
    public static double MAX_ANGULAR_VEL_RAD_PER_SEC = Math.PI;

    /**
     * Maximum angular acceleration (rad/s²). For quick turns.
     * Typical: 2π rad/s² (~360°/s²).
     */
    public static double MAX_ANGULAR_ACCEL_RAD_PER_SEC_SQ = 2 * Math.PI;

    // ========================================================================
    // PATH CONTROLLERS (Ramsete and PID-like)
    // ========================================================================
    /**
     * Ramsete B parameter (aggressiveness: >0, typical 1.5-3.0).
     * High: Follows path aggressively but may oscillate.
     */
    public static double RAMSETE_B = 2.0;

    /**
     * Ramsete Zeta parameter (damping: 0<zeta<1, typical 0.6-0.8).
     * High: Less oscillation, slower response.
     */
    public static double RAMSETE_ZETA = 0.7;

    /**
     * Turn gain (for heading control in turns: typical 0.1-0.5).
     */
    public static double TURN_GAIN = 0.25;

    /**
     * Turn velocity gain (0.0-0.5).
     */
    public static double TURN_VEL_GAIN = 0.1;

    // ========================================================================
    // TOLERANCES AND VERIFICATIONS (Pose Precision)
    // ========================================================================
    /**
     * Tolerance for XY position error (meters).
     * Typical: 0.02-0.05m for high precision.
     */
    public static double POSE_XY_TOLERANCE_METERS = 0.05;

    /**
     * Tolerance for heading error (radians). ~3°.
     */
    public static double HEADING_TOLERANCE_RADIANS = Math.toRadians(3.0);

    /**
     * Minimum distance to consider "arrival" at path end (pose error norm).
     */
    public static double PATH_END_TOLERANCE_METERS = 0.08;

    // ========================================================================
    // INITIALIZATION METHOD (Call in OpMode Init)
    // ========================================================================
    /**
     * Initializes and calculates all parameters based on provided hardware.
     * Call exactly once at the beginning of the OpMode (e.g., before creating drives/helpers).
     *
     * @param wheelDiameterInches Wheel diameter in inches (e.g., 4.0 for 4" wheel).
     * @param gearRatio Motor gear ratio (e.g., 20.0 for REV HD Hex 20:1).
     * @param encoderCPR Counts Per Revolution of the encoder on the motor shaft (e.g., 28 for REV built-in).
     * @param trackWidthInches Track width in inches (e.g., 18.0, measure center to center of wheels).
     *
     * Example Call:
     * DriveConstants.initialize(4.0, 20.0, 28, 18.0);
     *
     * Validations:
     * - Checks if values are positive.
     * - Calculates IN_PER_TICK = (π * wheelDiameterMeters) / (encoderCPR * gearRatio * 4)  // *4 for quadrature.
     * - TICKS_PER_REVOLUTION = encoderCPR * gearRatio * 4.
     * - WHEEL_RADIUS_METERS = (wheelDiameterInches / 2) * 0.0254 (inches to meters).
     * - TRACK_WIDTH_METERS = trackWidthInches * 0.0254.
     *
     * For REV HD Hex 20:1 + 4" wheel + 18" track:
     * - IN_PER_TICK ≈ 0.000142 m/tick.
     * - TICKS_PER_REVOLUTION = 2240.
     */
    public static void initialize(double wheelDiameterInches, double gearRatio, int encoderCPR, double trackWidthInches) {
        // Basic validations
        if (wheelDiameterInches <= 0 || gearRatio <= 0 || encoderCPR <= 0 || trackWidthInches <= 0) {
            throw new IllegalArgumentException("Hardware parameters must be positive!");
        }

        // Unit conversions (inches to meters)
        double wheelDiameterMeters = wheelDiameterInches * 0.0254;
        WHEEL_RADIUS_METERS = wheelDiameterMeters / 2.0;
        TRACK_WIDTH_METERS = trackWidthInches * 0.0254;

        // Calculation of ticks per revolution (output shaft): CPR * gearRatio * 4 (quadrature for direction/precision)
        TICKS_PER_REVOLUTION = encoderCPR * (int) gearRatio * 4;

        // Wheel circumference (meters per revolution)
        double wheelCircumferenceMeters = Math.PI * wheelDiameterMeters;

        // Meters per tick
        IN_PER_TICK = wheelCircumferenceMeters / TICKS_PER_REVOLUTION;

        // Log/Validation (can be used in telemetry)
        telemetry.addData("DriveConstants Initialized:", "See below");
        telemetry.addData("  Wheel Radius: ", WHEEL_RADIUS_METERS + " m");
        telemetry.addData("  Ticks/Rev: ", TICKS_PER_REVOLUTION);
        telemetry.addData("  IN_PER_TICK: ", IN_PER_TICK + " m/tick");
        telemetry.addData("  Track Width: ", TRACK_WIDTH_METERS + " m");

    }

    /**
     * Initialization with default values for REV HD Hex 20:1 + 4" wheel + 18" track.
     * Call if you don't want to specify: DriveConstants.initializeDefaults();
     */
    public static void initializeDefaults() {
        initialize(DEFAULT_WHEEL_DIAMETER_INCHES, DEFAULT_GEAR_RATIO, DEFAULT_ENCODER_CPR, DEFAULT_TRACK_WIDTH_INCHES);
    }

    // ========================================================================
    // AUXILIARY METHODS (For Additional Calculations)
    // ========================================================================
    /**
     * Converts velocity from m/s to ticks/s (for tick-based profiles, e.g., TankDrive).
     * @param velMetersPerSec Linear velocity (m/s)
     * @return Ticks per second
     */
    public static double metersPerSecToTicksPerSec(double velMetersPerSec) {
        return velMetersPerSec / IN_PER_TICK;
    }

    /**
     * Converts acceleration from m/s² to ticks/s².
     * @param accelMetersPerSecSq Acceleration (m/s²)
     * @return Ticks per second squared
     */
    public static double metersPerSecSqToTicksPerSecSq(double accelMetersPerSecSq) {
        return accelMetersPerSecSq / IN_PER_TICK;
    }

    /**
     * Checks if the calculated values are valid (e.g., IN_PER_TICK > 0).
     * @return true if initialized correctly
     */
    public static boolean isValid() {
        return IN_PER_TICK > 0 && TICKS_PER_REVOLUTION > 0 && WHEEL_RADIUS_METERS > 0 && TRACK_WIDTH_METERS > 0;
    }
}
