package ftc.team.allmight.plusultra.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.Java_Is_AllMight.Sensors.IMUMight;
import ftc.team.Java_Is_AllMight.Utils.Alliance;

/**
 * Rewritten Drive subsystem for higher precision:
 * - Uses RUN_WITHOUT_ENCODER and internal PID loops for distance + heading.
 * - Heading correction runs during linear motion.
 * - Improved turnIMU with ramping and correct PID call semantics.
 * - Global alliance mirroring via 'side' multiplier.
 * - Async move support retained (movePID-based).
 *
 * Assumes PIDController.calculate(setpoint, measurement) exists and returns control output.
 */
public class DriveTest {

    public final DcMotor leftMotor;
    public final DcMotor rightMotor;
    private final IMUMight imu;

    private Alliance alliance = Alliance.RED;
    private int side = 1; // 1 for RED, -1 for BLUE

    private static final double TICKS_PER_REV = 560.0;
    private static final double WHEEL_DIAMETER_CM = 9.0;
    private static final double WHEEL_CIRCUMFERENCE_CM = Math.PI * WHEEL_DIAMETER_CM;
    private static final double CM_PER_TICK = WHEEL_CIRCUMFERENCE_CM / TICKS_PER_REV;

    // Distance PID (control forward/backward)
    private final PIDController distPID = new PIDController(new PIDConfig(
            0.012,  // kP (start conservative)
            0.00001, // kI (small helps steady-state)
            0.001,   // kD
            0.0,     // kF (not used)
            20       // iZone (ticks)
    ));

    // Heading PID (used both for heading hold and turn control)
    private final PIDController headingPID = new PIDController(new PIDConfig(
            0.012,   // kP
            0.0,     // kI
            0.002,   // kD
            0.0,     // kF
            5        // iZone (degrees)
    ));

    // Turn-specific PID (tighter tuning)
    private final PIDController turnPID = new PIDController(new PIDConfig(
            0.08,   // kP
            0.0001, // kI
            0.0008, // kD
            0.0,
            10
    ));

    // Async move state
    private boolean asyncActive = false;
    private double asyncTargetCm = 0;
    private double asyncMaxPower = 0.6;
    private double asyncMinPower = 0.12;
    private long asyncStartTime = 0;
    private long asyncTimeoutMs = 0;
    private double asyncTargetHeading = 0;

    public DriveTest(HardwareMap hardwareMap, IMUMight imu) {
        this.imu = imu;

        leftMotor = hardwareMap.get(DcMotor.class, "motorEsquerdo");
        rightMotor = hardwareMap.get(DcMotor.class, "motorDireito");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // prefer direct power control
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void defineAlliance(Alliance alliance){
        this.alliance = alliance;
        this.side = alliance == Alliance.RED ? 1 : -1;
    }

    // -----------------------------
    // helpers
    // -----------------------------
    private int cmToTicks(double cm) {
        return (int) Math.round(cm / CM_PER_TICK);
    }

    private double getAverageDistanceTicks() {
        return ((double)leftMotor.getCurrentPosition() + (double)rightMotor.getCurrentPosition()) / 2.0;
    }

    public void resetEncodersToZeroPosition() {
        // This method tries to zero reference; because we're in RUN_WITHOUT_ENCODER
        // we'll use the current positions as reference (caller should use position offsets)
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        asyncActive = false;
    }

    // -----------------------------
    // Synchronous PID-driven move (blocking) with heading hold
    // -----------------------------
    public void moveToDistancePID(double cm, double maxPower, double timeoutMs, Telemetry telemetry) {
        // set reference
        resetEncodersToZeroPosition();
        int targetTicks = cmToTicks(cm);
        double startHeading = imu.getYaw();
        double targetHeading = startHeading; // hold initial heading by default

        distPID.reset();
        headingPID.reset();

        long deadline = System.currentTimeMillis() + (long)timeoutMs;

        while (System.currentTimeMillis() < deadline) {
            imu.update();

            double currentTicks = getAverageDistanceTicks();
            double errorTicks = targetTicks - currentTicks;

            // exit condition: within small threshold
            if (Math.abs(errorTicks) < 10) break;

            double forward = distPID.calculate(targetTicks, currentTicks); // setpoint, measurement

            // scale forward to power range
            forward = Range.clip(forward, -maxPower, maxPower);

            // heading correction
            double headingError = imu.normalizeAngle(targetHeading - imu.getYaw());
            double correction = headingPID.calculate(0, headingError); // want headingError -> 0

            // combine and apply
            double leftPower = Range.clip(forward + correction, -1, 1);
            double rightPower = Range.clip(forward - correction, -1, 1);

            // ensure minimum power to avoid stalling when far from zero
            if (Math.abs(leftPower) < 0.06) leftPower = Math.copySign(0.06, leftPower);
            if (Math.abs(rightPower) < 0.06) rightPower = Math.copySign(0.06, rightPower);

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // telemetry
            if (telemetry != null) {
                telemetry.addData("distErrorTicks", errorTicks);
                telemetry.addData("forward", forward);
                telemetry.addData("headingErr", headingError);
                telemetry.update();
            }

            // small sleep to yield
            try { Thread.sleep(10); } catch (InterruptedException ignored) {}
        }

        stop();
    }

    // -----------------------------
    // Async PID-driven move (non-blocking): start + update
    // -----------------------------
    public void moveToDistanceAsync(double cm, double maxPower, double timeoutMs) {
        resetEncodersToZeroPosition();
        asyncTargetCm = cm;
        asyncMaxPower = maxPower;
        asyncTimeoutMs = (long) timeoutMs;
        asyncTargetHeading = imu.getYaw();
        asyncStartTime = System.currentTimeMillis();

        distPID.reset();
        headingPID.reset();

        asyncActive = true;
    }

    public void moveToDistanceAsyncUpdate() {
        if (!asyncActive) return;

        if (System.currentTimeMillis() - asyncStartTime > asyncTimeoutMs) { stop(); return; }

        imu.update();
        double targetTicks = cmToTicks(asyncTargetCm);
        double currentTicks = getAverageDistanceTicks();
        double errorTicks = targetTicks - currentTicks;

        if (Math.abs(errorTicks) < 10) { stop(); return; }

        double forward = distPID.calculate(targetTicks, currentTicks);
        // map forward to acceptable power band
        forward = Range.clip(forward, -asyncMaxPower, asyncMaxPower);

        double headingError = imu.normalizeAngle(asyncTargetHeading - imu.getYaw());
        double correction = headingPID.calculate(0, headingError);

        double leftPower = Range.clip(forward + correction, -1, 1);
        double rightPower = Range.clip(forward - correction, -1, 1);

        // enforce minimum when moving
        if (Math.abs(leftPower) < asyncMinPower) leftPower = Math.copySign(asyncMinPower, leftPower);
        if (Math.abs(rightPower) < asyncMinPower) rightPower = Math.copySign(asyncMinPower, rightPower);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public boolean moveIsBusy() { return asyncActive; }

    // -----------------------------
    // Improved turnIMU (blocking) with ramping & timeout
    // -----------------------------
    public void turnIMU(double degrees, double maxPower, double toleranceDeg, Telemetry telemetry, long timeoutMs) {
        // apply alliance mirror
        degrees = degrees * side;

        imu.update();
        double start = imu.getYaw();
        double target = imu.normalizeAngle(start + degrees);

        turnPID.reset();

        long deadline = System.currentTimeMillis() + timeoutMs;

        while (System.currentTimeMillis() < deadline) {
            imu.update();
            double current = imu.getYaw();
            double error = imu.normalizeAngle(target - current);

            if (Math.abs(error) <= toleranceDeg) break;

            // PID expects setpoint, measurement; here setpoint = 0 (we want error->0) so pass accordingly
            double pidOut = turnPID.calculate(0, error);

            // ramp output based on error magnitude (smooth approach)
            double scale = Range.clip(Math.abs(error) / 40.0, 0.2, 1.0); // within 40 deg -> scale 0.2..1.0
            double out = Range.clip(pidOut * scale, -maxPower, maxPower);

            // ensure minimum to overcome stiction
            double min = 0.06;
            if (Math.abs(out) < min) out = Math.copySign(min, out);

            leftMotor.setPower(out);
            rightMotor.setPower(-out);

            if (telemetry != null) {
                telemetry.addData("turnError", error);
                telemetry.addData("turnOut", out);
                telemetry.update();
            }

            try { Thread.sleep(8); } catch (InterruptedException ignored) {}
        }

        stop();
    }

}
