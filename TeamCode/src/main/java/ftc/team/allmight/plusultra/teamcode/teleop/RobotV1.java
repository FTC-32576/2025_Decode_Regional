package ftc.team.allmight.plusultra.teamcode.teleop;

import ftc.team.allmight.plusultra.teamcode.commands.ShooterCommand;
import ftc.team.allmight.plusultra.teamcode.subsystems.ShooterSubsystem;
import ftc.team.Java_Is_AllMight.Logging.ChassisSpeed;
import ftc.team.Java_Is_AllMight.Logging.EnchancedLoggers.CustomChassisSpeedsLogger;
import ftc.team.Java_Is_AllMight.Sensors.IMUHelper;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotV1 extends OpMode {

    private DcMotor leftMotor, rightMotor;
    private CustomChassisSpeedsLogger chassisLogger;
    private IMUHelper imu;

    private double inputY, inputX;

    private ShooterSubsystem shooterSubsystem;
    private ShooterCommand shooterCommand;

    @Override
    public void init() {
        chassisLogger = new CustomChassisSpeedsLogger("ChassiLogger", telemetry);
        leftMotor = hardwareMap.get(DcMotor.class, "motorEsquerdo");
        rightMotor = hardwareMap.get(DcMotor.class, "motorDireito");

        // Shooter
        shooterSubsystem = new ShooterSubsystem(hardwareMap, "shooter", telemetry);
        shooterCommand = new ShooterCommand(shooterSubsystem);

        // IMU
        imu = new IMUHelper(hardwareMap, "imu", RevHubOrientationOnRobot.UsbFacingDirection.UP, RevHubOrientationOnRobot.LogoFacingDirection.FORWARD);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // ---------------------- Movimentação ----------------------
        inputY = -gamepad1.left_stick_y; //Frente tras
        inputX = gamepad1.right_stick_x; // Giro

        double leftPower = Range.clip(inputY + inputX, -1.0, 1.0);
        double rightPower = Range.clip(inputY - inputX, -1.0, 1.0);

        moveTank(leftPower, rightPower);

        // ---------------------- Log do Chassi ----------------------
        ChassisSpeed currentsSpeeds = new ChassisSpeed(inputY, 0, inputX);
        chassisLogger.append(currentsSpeeds);

        // ---------------------- Shooter ----------------------

        if (gamepad1.right_bumper) {
            shooterCommand.execute(1.0);
        }
//        } else if (gamepad1.right_trigger > 0.1) {
//            // RPM alvo proporcional ao gatilho
//            double targetRPM = 3500 * gamepad1.right_trigger; // exemplo: 0-3500 RPM
//            shooterCommand.executeRPM(targetRPM);
//        }
        else {
            shooterCommand.end();
        }

        // Atualiza PID a cada loop
        shooterCommand.update();

        // ---------------------- Telemetria ----------------------
        telemetry.addData("Shooter RPM", shooterSubsystem.getShooterRPM());
        telemetry.addData("Shooter Setpoint", shooterSubsystem.shooterTarget);
        telemetry.addData("Shooter AtSetpoint", shooterSubsystem.atSetpoint());
        telemetry.addData("IMU Yaw/Pitch/Roll", "%f / %f / %f", imu.getYaw(), imu.getPitch(), imu.getRoll());
        telemetry.update();
    }

    public void moveTank(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        ChassisSpeed currentsSpeeds = new ChassisSpeed(inputY, 0, inputX);
        chassisLogger.append(currentsSpeeds);

        telemetry.addData("Velocidade dos Motores", "Esquerda: %.2f | Direita: %.2f", leftPower, rightPower);
    }
}
