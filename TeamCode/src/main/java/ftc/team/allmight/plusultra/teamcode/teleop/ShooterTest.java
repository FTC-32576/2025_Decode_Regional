package ftc.team.allmight.plusultra.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.Java_Is_AllMight.Logging.ChassisSpeed;
import ftc.team.Java_Is_AllMight.Utils.RoboUtils;
import ftc.team.allmight.plusultra.teamcode.roadrunner.drive.SampleTankDrive;
import ftc.team.allmight.plusultra.teamcode.subsystems.Drive;
import ftc.team.allmight.plusultra.teamcode.subsystems.IntakeSubsytem;
import ftc.team.allmight.plusultra.teamcode.subsystems.ServoSubsystem;
import ftc.team.allmight.plusultra.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Subsystem Teste REAL")
public class ShooterTest extends OpMode {

//    private ShooterSubsystem shooter;
    private DcMotorEx shooter;

    private IntakeSubsytem intake;
    private ServoSubsystem servo;
    private Drive drive;

    private SampleTankDrive odometryTank;

    private static final double TICKS_PER_REV = 28.0;
    private static final double GEARS = 25.0 / 10.0;


    private PIDConfig pidShooterSettings = new PIDConfig(
//            0.0008,
//            0.00002,
//            0.0001
            0.00038,
            0.000001,
            0.0001
    );

    private PIDController pidShooter = new PIDController(pidShooterSettings);

    private static final double potenciaAlvo = 660;

    @Override
    public void init() {
//        shooter = new ShooterSubsystem();
//        shooter.init(hardwareMap);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        intake = new IntakeSubsytem(hardwareMap, telemetry);
        servo = new ServoSubsystem(hardwareMap);
        drive = new Drive(hardwareMap);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odometryTank = new SampleTankDrive(hardwareMap);
    }

    @Override
    public void loop() {

//        odometryTank.update();


        // Atualiza shooter
//        shooter.update(telemetry);

        // INTake
        if (gamepad2.right_trigger > 0.1) intake.intake();
        else if (gamepad2.left_trigger > 0.1) intake.reverse();
        else intake.stop();

        intake.update();

        // Shooter Power / Start / Stop
//        if (gamepad2.right_bumper) {
//            double targetRpm = 3200;          // RPM desejado da saída
//            double targetRpmMotor  = targetRpm / GEARS;  // RPM no motor
//
//            double ticksPerSecond = rpmToTicksPerSecond(targetRpmMotor);
//            shooter.getVelocity();
//            shooter.setPower(0.2548);
//            pidShooter.calculate(setpoint,measurement);
////            shooter.setVelocity(ticksPerSecond);
//        }
//        if (gamepad2.left_bumper) shooter.setVelocity(0);

        if (gamepad2.right_bumper) {

            double velocidadeAtual = shooter.getVelocity();   // ticks/seg atual
            double velocidadeAlvo = potenciaAlvo;             // já veio como 660 (ticks/s)

            double erro = velocidadeAlvo - velocidadeAtual;

            double pidOutput = pidShooter.calculate(velocidadeAlvo, velocidadeAtual);

            // Ajuste final de potência
            double potencia = Range.clip(pidOutput, 0, 1);

            shooter.setPower(potencia);

            telemetry.addData("PID Output", pidOutput);
            telemetry.addData("Erro", erro);
            telemetry.addData("Potência final", potencia);
        }

        if (gamepad2.left_bumper) {
            shooter.setPower(0);
        }

        // Servo -> posições fixas
        if (gamepad2.x) {
            servo.setPosition(0.345); // disparar
        } else if(gamepad2.y){
            servo.setPosition(0.0);   // fechado
        }



        // Drive
        drive.drive(
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        telemetry.addData("Servo Pos", servo.getPosition());
        telemetry.addData("Velocidade do Shooter", shooter.getVelocity());
        telemetry.update();
    }

    public double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }
}
