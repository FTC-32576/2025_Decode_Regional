package ftc.team.allmight.plusultra.teamcode.teleop;

import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.allmight.plusultra.teamcode.roadrunner.drive.SampleTankDrive;
import ftc.team.allmight.plusultra.teamcode.subsystems.IntakeSubsytem;
import ftc.team.allmight.plusultra.teamcode.subsystems.ShooterSubsystem;
import ftc.team.Java_Is_AllMight.Logging.ChassisSpeed;
import ftc.team.Java_Is_AllMight.Logging.EnchancedLoggers.CustomChassisSpeedsLogger;
import ftc.team.Java_Is_AllMight.Sensors.IMUMight;
import ftc.team.Java_Is_AllMight.Utils.Alliance;
import ftc.team.allmight.plusultra.teamcode.utils.AutoAimUtils;
import ftc.team.allmight.plusultra.teamcode.utils.FieldUtils;
import ftc.team.Java_Is_AllMight.Utils.RoboUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "RoboV1")
public class RobotV1 extends OpMode {

    private enum AimState {
        DRIVING,
        AIMING,
        AIMED
    }

    private AimState aimState = AimState.DRIVING;

    public Servo servo;
    private DcMotor leftMotor, rightMotor;
    private IntakeSubsytem intakeMotor;
    private ShooterSubsystem shooterSubsystem;
    private CustomChassisSpeedsLogger chassisLogger;
    private IMUMight imu;
    private SampleTankDrive drive;

    private final Alliance alliance = Alliance.BLUE;
    private final RoboUtils roboUtils = new RoboUtils();
    private PIDConfig turnPIDConfig = new PIDConfig(0.03, 0.0, 0.002);

    private double inputY, inputX;

    @Override
    public void init() {
        chassisLogger = new CustomChassisSpeedsLogger("ChassiLogger", telemetry);
        leftMotor = roboUtils.getHardware(hardwareMap, DcMotor.class, "motorEsquerdo");
        rightMotor = roboUtils.getHardware(hardwareMap, DcMotor.class, "motorDireito");
        intakeMotor = new IntakeSubsytem(hardwareMap, telemetry);
        servo = hardwareMap.get(Servo.class, "servidor");

        shooterSubsystem = new ShooterSubsystem();
        shooterSubsystem.init(hardwareMap);

//        drive = new SampleTankDrive(hardwareMap);

        imu = new IMUMight(hardwareMap, "imu",
                RevHubOrientationOnRobot.UsbFacingDirection.UP,
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD);

//        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Inicializa pose do RR - PELO AMOR DE DEUS COLOCA ELE NO CANTO INFERIOR ESQUERDO VIRADO PRO GOAL SENAO NAO VAI FUNCIONARRRRRRR
//        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        telemetry.addLine("PRONTO Po ✅");
        telemetry.update();
    }



    @Override
    public void loop() {
        imu.update();
//        drive.update();
//
//        if(gamepad1.left_bumper) intakeMotor.intake(); else if(gamepad1.right_bumper) intakeMotor.reverse(); else intakeMotor.stop();
//        intakeMotor.update();
//
//        boolean shooterButton = gamepad1.right_trigger > 0.1;
//
//        if(shooterButton) shooterSubsystem.shoot(); else shooterSubsystem.stop(); //RPM
//        boolean aimingButton = gamepad1.a;
//
//        shooterSubsystem.update(telemetry);
//
//        switch (aimState) {
//
//            case DRIVING:
//                driveManual();
//                if (aimingButton) {
//                    aimState = AimState.AIMING;
//                }
//                break;
//
//            case AIMING:
//                pararMotores();
//
//                boolean aligned = AutoAimUtils.aimAjust(
//                        null, null, alliance, drive,
//                        leftMotor, rightMotor,
//                        turnPIDConfig, 0.5
//                );
//
//                if (aligned) aimState = AimState.AIMED;
//                if (!aimingButton) aimState = AimState.DRIVING;
//                break;
//
//            case AIMED:
//                pararMotores();
//                telemetry.addLine("Alinhado");
//
//                if(shooterSubsystem.isReadyToShoot()){
//                    telemetry.addLine("Ta pronto pra shoot");
//                } else{
//                    telemetry.addLine("Alinhando");
//                }
//                if (!aimingButton) aimState = AimState.DRIVING;
//                break;
//        }
//
//        controlarIntake();
//        mostrarTelemetry();

//        intake.update();
//
//        telemetry.addData("Corrente", shooter.getCurrent(CurrentUnit.AMPS));
//        telemetry.addLine(String.valueOf(servo.getPosition()));
//        telemetry.addLine(servo.getDirection().toString());
//        if(gamepad1.right_bumper){
////            shooter.shoot();
//            shooter.setPower(1);
//
//        } else if(gamepad1.left_bumper){
////            shooter.stop();
//            shooter.setPower(0);
//        }
//
//        if(gamepad1.y){
//            controlarServo(0.0);
//
//        }else if (gamepad1.x) {
//            controlarServo(0.5);
//        }
//
//        if(gamepad1.right_trigger > 0.1){
//            intake.intake();
//        } else {
//            intake.stop();
//        }


        telemetry.update();
    }

    public void controlarServo(double direcao){
        if(direcao == 0.0){
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(direcao);
        } else if (direcao == 0.5) {
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(direcao);

        }
    }

    private void driveManual() {
        inputY = -gamepad1.left_stick_y;
        inputX = gamepad1.right_stick_x;

        double leftPower = Range.clip(inputY + inputX, -1, 1);
        double rightPower = Range.clip(inputY - inputX, -1, 1);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        chassisLogger.append(new ChassisSpeed(inputY, 0, inputY));
    }

    private void pararMotores() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void controlarIntake() {
        if (gamepad1.left_bumper) intakeMotor.intake();
        else if (gamepad1.right_bumper) intakeMotor.reverse();
        else intakeMotor.stop();

        intakeMotor.update();
    }

    private void mostrarTelemetry() {
        telemetry.addData("Yaw", imu.getYaw());
        telemetry.addData("Estado", aimState);
//        telemetry.addData("Pose", drive.getPoseEstimate().toString());
        telemetry.update();
    }
}
