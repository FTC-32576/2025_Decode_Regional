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

    private DcMotor shooterMotor, leftMotor, rightMotor;
    private CustomChassisSpeedsLogger chassisLogger;
    private IMUHelper imu;

    private final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    private final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    private double inputY, inputX;
    private YawPitchRollAngles robotOrientation;

    double Heading, Pitch,Roll;

    private ShooterSubsystem shooterSubsystem;
    private ShooterCommand shooterCommand;

    @Override
    public void init() {
        chassisLogger = new CustomChassisSpeedsLogger("ChassiLogger", telemetry);
        leftMotor = getHardware(DcMotor.class, "motorEsquerdo");
        rightMotor = getHardware(DcMotor.class, "motorDireito");
        shooterSubsystem = new ShooterSubsystem(hardwareMap, "shooter");
        shooterCommand = new ShooterCommand(shooterSubsystem);

        imu = new IMUHelper(hardwareMap, "imu", RevHubOrientationOnRobot.UsbFacingDirection.UP, RevHubOrientationOnRobot.LogoFacingDirection.FORWARD);

        Heading = imu.getYaw();
        Pitch = imu.getPitch();
        Roll = imu.getRoll();

        telemetry.addData("Atributos do IMU", "Yaw: %s, Pitch %s, Roll %s", Heading, Pitch, Roll);
        telemetry.update();

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        inputY = -gamepad1.left_stick_y;
        inputX = gamepad1.right_stick_x;

        double leftPower, rightPower;

        leftPower = Range.clip(inputY + inputX, -1.0, 1.0);
        rightPower = Range.clip(inputY - inputX, -1.0, 1.0);

        this.moveTank(leftPower, rightPower);


        if(gamepad1.right_bumper){
            shooterCommand.execute(1);
        }

        telemetry.addData("Atributos do IMU", "Yaw: %s, Pitch %s, Roll %s", imu.getYaw(), imu.getPitch(), imu.getRoll());
        telemetry.update();

    }

    public <T> T getHardware(Class<T> tipoDeHardware, String nome){
        return hardwareMap.get(tipoDeHardware, nome);

    }

    public void moveTank(double leftPower, double rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        ChassisSpeed currentsSpeeds = new ChassisSpeed(inputY, 0, inputY);

        chassisLogger.append(currentsSpeeds);

        telemetry.addData("Velocidade dos Motores", "Velocidade Esquerda: %s | Velocidade Direita %s", leftPower, rightPower);
        telemetry.update();
    }

}

