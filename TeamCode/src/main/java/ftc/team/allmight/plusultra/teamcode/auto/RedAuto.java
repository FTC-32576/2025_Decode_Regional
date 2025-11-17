package ftc.team.allmight.plusultra.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import ftc.team.allmight.plusultra.teamcode.subsystems.Drive;
import ftc.team.Java_Is_AllMight.Sensors.IMUMight;
import ftc.team.allmight.plusultra.teamcode.subsystems.ServoSubsystem;

@Autonomous(name = "AUTO - Simples", group = "A")
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // IMU
        IMUMight imu = new IMUMight(hardwareMap, "imu", RevHubOrientationOnRobot.UsbFacingDirection.UP, RevHubOrientationOnRobot.LogoFacingDirection.RIGHT);
        imu.resetYaw();

        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo servo = hardwareMap.get(Servo.class, "servidor");

        // DRIVE
        Drive drive = new Drive(hardwareMap, imu);

        telemetry.addLine("Pronto");
        telemetry.update();

        waitForStart();


        if (isStopRequested()) return;

        // 1) anda para frente 60 cm
        drive.moveSmooth(25, 0.55);

//        cameraMight.alignCenterTag(drive, id, this);
        // 2) gira 90° para a direita
//        drive.turnIMU(-10, 0.33, 2.5, telemetry);

        shootar(3,0.345, 0, shooter, servo);
        // ENCERRA
        drive.stop();

        while(opModeIsActive()) {
            telemetry.addData("IMU", imu.getYaw());
            telemetry.update();
        }


    }

    public void shootar(int shots, double posX, double posY, DcMotorEx shooter, Servo servo){
        shooter.setPower(1);

        for (int i =0; i <= shots; i++){
            sleep(2000);

            servo.setPosition(posX);
            sleep(800);

            servo.setPosition(posY);
        }

        shooter.setPower(0);
    }
}
