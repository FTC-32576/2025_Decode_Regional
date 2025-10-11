package ftc.team.allmight.plusultra.teamcode.teleop;

import ftc.team.java_is_allmight.Logging.EnchancedLoggers.CustomChassisSpeedsLogger;
import ftc.team.java_is_allmight.Sensors.IMUHelper;

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

    private YawPitchRollAngles robotOrientation;

    @Override
    public void init() {
        chassisLogger = new CustomChassisSpeedsLogger("ChassiLogger", telemetry);
        leftMotor = getHardware(DcMotor.class, "motorEsquerdo");
        rightMotor = getHardware(DcMotor.class, "motorDireito");
        shooterMotor = getHardware(DcMotor.class, "shooter");
        imu = new IMUHelper(hardwareMap, "imu", RevHubOrientationOnRobot.UsbFacingDirection.UP, RevHubOrientationOnRobot.LogoFacingDirection.FORWARD);

        double Heading = imu.getYaw();
        double Pitch = imu.getPitch();
        double Roll = imu.getRoll();

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

        moveTank(leftPower, rightPower);


    }

    public <T> T getHardware(Class<T> tipoDeHardware, String nome){
        return hardwareMap.get(tipoDeHardware, nome);

    }

    public void moveTank(double leftPower, double rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("Velocidade dos Motores", "Velocidade Esquerda: %s | Velocidade Direita %s", leftPower, rightPower);
        telemetry.update();
    }

}

