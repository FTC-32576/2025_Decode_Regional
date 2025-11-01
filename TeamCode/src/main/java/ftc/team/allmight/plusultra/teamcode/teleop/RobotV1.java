package ftc.team.allmight.plusultra.teamcode.teleop;

import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Sensors.LimeMight;
import ftc.team.allmight.plusultra.teamcode.roadrunner.drive.SampleTankDrive;
import ftc.team.allmight.plusultra.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import ftc.team.allmight.plusultra.teamcode.subsystems.IntakeSubsytem;
import ftc.team.allmight.plusultra.teamcode.subsystems.ShooterSubsystem;
import ftc.team.Java_Is_AllMight.Logging.ChassisSpeed;
import ftc.team.Java_Is_AllMight.Logging.EnchancedLoggers.CustomChassisSpeedsLogger;
import ftc.team.Java_Is_AllMight.Sensors.IMUMight;
import ftc.team.Java_Is_AllMight.Utils.Alliance;
import ftc.team.allmight.plusultra.teamcode.utils.AutoAimUtils;
import ftc.team.allmight.plusultra.teamcode.utils.FieldUtils;
import ftc.team.Java_Is_AllMight.Utils.RoboUtils;
import ftc.team.allmight.plusultra.teamcode.utils.MathUtils;
import ftc.team.allmight.plusultra.teamcode.utils.ResultadoMira;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "RoboV1")
public class RobotV1 extends OpMode {

    private enum AimState {
        DRIVING,
        AIMING,
        AIMED
    }

    private AimState aimState = AimState.DRIVING;

    private DcMotor leftMotor, rightMotor;
    private IntakeSubsytem intakeMotor;
    private CustomChassisSpeedsLogger chassisLogger;
    private IMUMight imu;

    private final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    private final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    private SampleTankDrive drive;

    private double inputY, inputX;
    private YawPitchRollAngles robotOrientation;

    double Heading, Pitch,Roll;

    private IMU imu2;
    private ShooterSubsystem shooterSubsystem;

    // Coordenadas do GOAL (em centímetros)
    // Ajuste conforme a posição real no campo
    private  Pose2d GOAL_POSE;
    private Alliance alliance = Alliance.BLUE;
    private RoboUtils roboUtils = new RoboUtils();
    private Limelight3A limelight3A;

    private RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    LimeMight lime = new LimeMight(hardwareMap,"limelight", "imu", usbFacingDirection, logoFacingDirection, new PIDConfig(0.3, 0.001, 0.5), null);

    PIDConfig turnPIDConfig = new PIDConfig(0.03, 0.0, 0.002);



    @Override
    public void init() {
        chassisLogger = new CustomChassisSpeedsLogger("ChassiLogger", telemetry);
        leftMotor = roboUtils.getHardware(hardwareMap,DcMotor.class, "motorEsquerdo");
        rightMotor = roboUtils.getHardware(hardwareMap, DcMotor.class, "motorDireito");
        intakeMotor = new IntakeSubsytem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        drive = new SampleTankDrive(hardwareMap);
        imu = new IMUMight(hardwareMap, "imu", usbDirection , logoFacingDirection);


        Heading = imu.getYaw();
        Pitch = imu.getPitch();
        Roll = imu.getRoll();

        telemetry.addData("Atributos do IMU", "Yaw: %s, Pitch %s, Roll %s", Heading, Pitch, Roll);
        telemetry.update();



        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        GOAL_POSE = FieldUtils.getGoalPose(Alliance.BLUE);

        //COLOCA O ROBO VIRADO PRA FRENTE  (OLHANDO PRO GOAL) NO CANTO INFERIOR ESQUERDO, SENAO NAO VAI FUNCIONAR!!!
        drive.setPoseEstimate(new Pose2d(0,0, 0));



    }

    @Override
    public void loop() {
        drive.update();

        inputY = -gamepad1.left_stick_y;
        inputX = gamepad1.right_stick_x;

        double leftPower, rightPower;

        leftPower = Range.clip(inputY + inputX, -1.0, 1.0);
        rightPower = Range.clip(inputY - inputX, -1.0, 1.0);

        if (aimState != AimState.DRIVING) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        this.moveTank(leftPower, rightPower);

        if(gamepad1.left_bumper) intakeMotor.intake(); else if(gamepad1.right_bumper) intakeMotor.reverse(); else intakeMotor.stop();
        intakeMotor.update();

        boolean shooterButton = gamepad1.right_trigger > 0.1;

        if(shooterButton) shooterSubsystem.setTargetVelocity(6000); else shooterSubsystem.stop(); //RPM
        boolean aimingButton = gamepad1.a;

        shooterSubsystem.update();

        switch (aimState) {

            case DRIVING:
                if (aimingButton) {
                    aimState = AimState.AIMING;
                }
                break;

            case AIMING:
                boolean aligned = AutoAimUtils.aimAjust(
                        null, alliance, drive, leftMotor, rightMotor, turnPIDConfig, 0.5
                );

                if (aligned) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    aimState = AimState.AIMED;
                }

                if (!aimingButton) {
                    aimState = AimState.DRIVING;
                }
                break;

            case AIMED:
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                telemetry.addLine("Alinhado! Pronto para atirar!");

                if (shooterSubsystem.isReadyToShoot()) {
                    telemetry.addLine("Shooter pronto!");
                } else {
                    telemetry.addLine("Shooter acelerando...");
                }

                if (!aimingButton) {
                    aimState = AimState.DRIVING;
                }
                break;
        }


        telemetry.addData("Atributos do IMU", "Yaw: %s, Pitch %s, Roll %s", imu.getYaw(), imu.getPitch(), imu.getRoll());
        intakeMotor.update();
        telemetry.update();




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

