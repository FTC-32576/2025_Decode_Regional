package ftc.team.allmight.plusultra.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.Java_Is_AllMight.Sensors.IMUMight;

public class Drive {

    public final DcMotor leftMotor;
    public final DcMotor rightMotor;
    private final IMUMight imu;

    private static final double TICKS_PER_REV = 560.0;
    private static final double WHEEL_DIAMETER_CM = 9.0;
    private static final double WHEEL_CIRCUMFERENCE_CM = Math.PI * WHEEL_DIAMETER_CM;

    private static final double CM_PER_TICK = WHEEL_CIRCUMFERENCE_CM / TICKS_PER_REV;


    private final PIDController turnPID = new PIDController(
            new PIDConfig(
                    0.1,   // kP
                    0.00001,  // kI
                    0.0005, // kD
                    0.0,    // kF
                    10      // iZone (10°)
            )
    );

    public Drive(HardwareMap hardwareMap, IMUMight imu) {
        this.imu = imu;

        leftMotor = hardwareMap.get(DcMotor.class, "motorEsquerdo");
        rightMotor = hardwareMap.get(DcMotor.class, "motorDireito");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // -----------------------------
    // TANK DRIVE NORMAL
    // -----------------------------
    public void drive(double forward, double turn) {
        double leftPower  = Range.clip(forward + turn, -1.0, 1.0);
        double rightPower = Range.clip(forward - turn, -1.0, 1.0);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    // -----------------------------
    // ENCODER HELPERS
    // -----------------------------
    private int cmToTicks(double cm) {
        return (int) (cm / CM_PER_TICK);
    }

    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBusy() {
        return leftMotor.isBusy() || rightMotor.isBusy();
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    // -----------------------------
    // MOVE COM ACELERAÇÃO E DESACELERAÇÃO
    // -----------------------------
    public void moveSmooth(double cm, double maxPower) {

        resetEncoders();

        int ticks = cmToTicks(cm);
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + ticks);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + ticks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double minPower = 0.12;      // evita morto
        double accelRate = 0.02;     // aceleração por loop
        double decelStart = 0.7;     // quando começar a desacelerar (%)

        double power = minPower;

        while (isBusy()) {
            double progress =
                    Math.abs((double)leftMotor.getCurrentPosition() /
                            (double)leftMotor.getTargetPosition());

            // ACELERAÇÃO
            if (progress < 0.3) {
                power += accelRate;
                if (power > maxPower) power = maxPower;
            }
            // DESACELERAÇÃO
            else if (progress > decelStart) {
                power -= accelRate;
                if (power < minPower) power = minPower;
            }

            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }

        stop();
    }

    // -----------------------------
    // GIRO PRECISO COM IMU
    // -----------------------------
    // -----------------------------
    public void turnIMU(double degrees, double maxPower, double tolerance, Telemetry telemetry) {

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.update();
        double start = imu.getYaw();
        double target = imu.normalizeAngle(start + degrees);

        turnPID.reset();

        while (true) {

            imu.update();
            double current = imu.getYaw();

            // erro normalizado entre -180 e 180
            double error = imu.normalizeAngle(target - current);

            if (Math.abs(error) <= tolerance)
                break;

            double pidOutput = turnPID.calculate(0, error);
            // setpoint = 0 → queremos erro = 0

            // limita potência
            pidOutput = Range.clip(pidOutput, -maxPower, maxPower);

            // mínima potência pra não travar
            double min = 0.07;
            if (Math.abs(pidOutput) < min)
                pidOutput = Math.copySign(min, pidOutput);

            leftMotor.setPower(pidOutput);
            rightMotor.setPower(-pidOutput);

            telemetry.addData("Yaw", current);
            telemetry.addData("Target", target);
            telemetry.addData("Error", error);
            telemetry.addData("PID", pidOutput);
            telemetry.update();
        }

        stop();
    }

    // -----------------------------
// MOVE SMOOTH ASSÍNCRONO (não bloqueia)
// -----------------------------
    private double async_maxPower;
    private double async_minPower = 0.12;
    private double async_accelRate = 0.02;
    private double async_decelStart = 0.6;

    private boolean async_active = false;

    public void moveSmoothStart(double cm, double maxPower) {

        resetEncoders();

        int ticks = cmToTicks(cm);
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + ticks);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + ticks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        async_maxPower = maxPower;
        async_active = true;
    }


    public void moveSmoothUpdate() {
        if (!async_active) return;

        if (!isBusy()) {
            stop();
            async_active = false;
            return;
        }

        double progress =
                Math.abs((double)leftMotor.getCurrentPosition() /
                        (double)leftMotor.getTargetPosition());

        double power = leftMotor.getPower();

        // Aceleração
        if (progress < 0.3) {
            power += async_accelRate;
            if (power > async_maxPower) power = async_maxPower;
        }
        // Desaceleração
        else if (progress > async_decelStart) {
            power -= async_accelRate;
            if (power < async_minPower) power = async_minPower;
        }

        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public boolean moveSmoothIsBusy() {
        return async_active;
    }


}
