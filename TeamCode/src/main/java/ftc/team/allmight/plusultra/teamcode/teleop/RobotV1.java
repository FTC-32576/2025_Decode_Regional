package ftc.team.allmight.plusultra.teamcode.teleop;

import ftc.team.java_is_allmight.Logging.ChassisSpeedsLogEntry;
import ftc.team.java_is_allmight.Logging.EnchancedLoggers.CustomChassisSpeedsLogger;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotV1 extends OpMode {

    private DcMotor shooterMotor, leftMotor, rightMotor;
    private CustomChassisSpeedsLogger chassisLogger;
    private IMU imu;

    private RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    private RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    private YawPitchRollAngles robotOrientation;

    @Override
    public void init() {
        chassisLogger = new CustomChassisSpeedsLogger("ChassiLogger", telemetry);
        leftMotor = encontrarHardware(DcMotor.class, "motorEsquerdo");
        rightMotor = encontrarHardware(DcMotor.class, "motorDireito");
        shooterMotor = encontrarHardware(DcMotor.class, "shooter");
        imu = encontrarHardware(IMU.class, "imu");

        IMU.Parameters imuParameters = new IMU.Parameters( new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection));
        imu.initialize(imuParameters);
        imu.resetYaw();
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double Heading = robotOrientation.getYaw();
        double Pitch = robotOrientation.getPitch();
        double Roll = robotOrientation.getRoll();

        telemetry.addData("Atributos do IMU", "Yaw: %s, Pitch %s, Roll %s", Heading, Pitch, Roll);
        telemetry.update();

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double leftPower, rightPower;

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        move_tank(leftPower, rightPower);

    }

    public <T> T encontrarHardware(Class<T> tipoDeHardware, String nome){
        return hardwareMap.get(tipoDeHardware, nome);

    }

    public void move_tank(double leftPower, double rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("Velocidade dos Motores", "Velocidade Esquerda: %s | Velocidade Direita %s", leftPower, rightPower);
        telemetry.update();
    }

}

