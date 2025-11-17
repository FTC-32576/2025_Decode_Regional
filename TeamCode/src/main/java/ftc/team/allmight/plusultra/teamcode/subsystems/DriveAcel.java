package ftc.team.allmight.plusultra.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import ftc.team.Java_Is_AllMight.Sensors.IMUMight;

public class DriveAcel {

    private final DcMotor leftMotor, rightMotor;
    private final IMUMight imu;

    // Ramping
    private double currentForward = 0;
    private final double rampRate = 0.08;

    // Heading target
    private double targetAngle = 0;

    public DriveAcel(HardwareMap hardwareMap) {

        leftMotor = hardwareMap.get(DcMotor.class, "motorEsquerdo");
        rightMotor = hardwareMap.get(DcMotor.class, "motorDireito");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new IMUMight(
                hardwareMap,
                "imu",

                RevHubOrientationOnRobot.UsbFacingDirection.UP,
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
        );
    }

    private double angleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void drive(double forward, double rightStickX, double rightStickY) {

        imu.update();
        double currentHeading = imu.getYaw();

        if (Math.abs(forward) < 0.01) {
            currentForward = 0;
        } else {
            double delta = forward - currentForward;
            currentForward += Range.clip(delta, -rampRate, rampRate);
        }

        double stickX = rightStickX;
        double stickY = -rightStickY;

        if (Math.hypot(stickX, stickY) > 0.15) {
            targetAngle = Math.toDegrees(Math.atan2(stickX, stickY));
        }


        double error = angleWrap(targetAngle - currentHeading);

        double turnPower;
        if (Math.abs(error) > 10) {
            turnPower = Range.clip(error / 90.0, -1.0, 1.0);
        } else {
            turnPower = error * 0.04;
        }

        // -----------------------------------------------------
        // 4. APLICAR AO DRIVE
        // -----------------------------------------------------
        double leftPower  = Range.clip(currentForward + turnPower, -1, 1);
        double rightPower = Range.clip(currentForward - turnPower, -1, 1);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }


    public void resetHeading() {
        imu.resetYaw();
        imu.update();
        targetAngle = imu.getYaw();
    }
}
