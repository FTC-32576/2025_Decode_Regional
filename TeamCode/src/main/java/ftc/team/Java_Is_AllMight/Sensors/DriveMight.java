package ftc.team.Java_Is_AllMight.Sensors;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.Java_Is_AllMight.Sensors.CameraMight;
import ftc.team.Java_Is_AllMight.Sensors.OdometryMight;
import ftc.team.Java_Is_AllMight.Utils.DriveConstants;

public class DriveMight {

    private DcMotor left, right;
    private OdometryMight odom;
    private CameraMight camera;

    private PIDController pidForward;
    private PIDController pidTurn;

    // Motion profile
    private final double MAX_VEL = DriveConstants.MAX_VEL;
    private final double MAX_ACC = DriveConstants.MAX_ACC;
    private final double DECEL_DIST = DriveConstants.DECEL_DIST;

    public DriveMight(HardwareMap hw, OdometryMight odom, CameraMight camera) {

        this.odom = odom;
        this.camera = camera;

        left  = hw.get(DcMotor.class, DriveConstants.LEFT_MOTOR);
        right = hw.get(DcMotor.class, DriveConstants.RIGHT_MOTOR);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidForward = new PIDController(DriveConstants.FWD_PID);
        pidTurn = new PIDController(DriveConstants.TURN_PID);
    }

    // ============================================================================
    // MOTION PROFILE HELPER
    // ============================================================================

    private double computeMotionSpeed(double remainingDist) {

        // Desacelera
        if (remainingDist <= DECEL_DIST) {
            double slowDownFactor = remainingDist / DECEL_DIST;
            return Math.max(0.1, MAX_VEL * slowDownFactor);
        }

        // Acelera até a velocidade máxima
        return MAX_VEL;
    }

    // ============================================================================
    // FORWARD
    // ============================================================================

    public void forward(double cm) {
        odom.resetDistance();

        pidForward.reset();

        double target = cm;

        while (!Thread.currentThread().isInterrupted()) {

            double traveled = odom.getForwardDistance();
            double remaining = Math.abs(target - traveled);
            if (remaining < 0.8) break;

            double baseSpeed = computeMotionSpeed(remaining);
            double correction = pidForward.calculate(target, traveled);

            double power = clamp(correction * baseSpeed);

            applyPower(power, power);
        }

        stop();
    }

    public void backward(double cm) {
        forward(-cm);
    }

    // ============================================================================
    // TURN
    // ============================================================================

    public void turn(double degrees) {

        odom.resetHeading();
        pidTurn.reset();

        while (!Thread.currentThread().isInterrupted()) {

            double heading = odom.getHeading();
            double remaining = Math.abs(degrees - heading);
            if (remaining < 1.0) break;

            double baseSpeed = computeMotionSpeed(remaining);
            double correction = pidTurn.calculate(degrees, heading);

            double p = clamp(correction * baseSpeed);

            applyPower(p, -p);
        }

        stop();
    }

    // ============================================================================
    // ALIGN WITH APRILTAG (PRECISÃO MÁXIMA)
    // ============================================================================

    public boolean alignTag(int id) {

        if (!camera.isTagVisible(id)) return false;

        PIDController pidX = new PIDController(DriveConstants.ALIGN_X_PID);
        PIDController pidY = new PIDController(DriveConstants.ALIGN_Y_PID);
        PIDController pidYaw = new PIDController(DriveConstants.ALIGN_YAW_PID);
        PIDController pidDist = new PIDController(DriveConstants.ALIGN_DIST_PID);

        while (!Thread.currentThread().isInterrupted()) {

            if (!camera.isTagVisible(id)) break;

            double tx = camera.getTx(id);
            double ty = camera.getTy(id);
            double yawErr = camera.getYaw(id);
            double distErr = camera.getDistanceToTag(id) - DriveConstants.DESIRED_DIST;

            double xPower = pidX.calculate(0, tx);
            double yPower = pidY.calculate(0, ty);
            double yawPower = pidYaw.calculate(0, yawErr);
            double fwdPower = pidDist.calculate(0, distErr);

            // Tank: só fwd + yaw
            double leftP  = clamp(fwdPower + yawPower);
            double rightP = clamp(fwdPower - yawPower);

            applyPower(leftP, rightP);

            if (Math.abs(tx) < 0.7 &&
                    Math.abs(ty) < 0.7 &&
                    Math.abs(yawErr) < 1.0 &&
                    Math.abs(distErr) < 1.0)
                break;
        }

        stop();
        return true;
    }

    // ============================================================================
    // POWER HELPERS
    // ============================================================================

    private void applyPower(double l, double r) {
        left.setPower(clamp(l));
        right.setPower(clamp(r));
    }

    private double clamp(double p) {
        return Math.max(-1, Math.min(1, p));
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }


}
