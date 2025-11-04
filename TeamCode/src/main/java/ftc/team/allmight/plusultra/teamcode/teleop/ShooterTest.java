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

import ftc.team.Java_Is_AllMight.Logging.ChassisSpeed;
import ftc.team.Java_Is_AllMight.Utils.RoboUtils;
import ftc.team.allmight.plusultra.teamcode.subsystems.IntakeSubsytem;
import ftc.team.allmight.plusultra.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Substyem Teste REAL")
public class ShooterTest extends OpMode {

//    ShooterSubsystem shooter = new ShooterSubsystem();

    private double inputY, inputX;
    private DcMotorEx shooter;
    private Servo servo;
    private IntakeSubsytem intake;
    private RoboUtils roboUtils;
    private DcMotor leftMotor, rightMotor;

    @Override
    public void init() {
//        shooter.init(hardwareMap);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        servo = hardwareMap.get(Servo.class, "servidor");
        leftMotor = hardwareMap.get(DcMotor.class, "motorEsquerdo");
        rightMotor = hardwareMap.get(DcMotor.class, "motorDireito");
        intake = new IntakeSubsytem(hardwareMap, telemetry);
        servo.setPosition(0);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        intake.update();

        telemetry.addData("Corrente", shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine(String.valueOf(servo.getPosition()));
        telemetry.addLine(servo.getDirection().toString());
        if(gamepad1.right_bumper){
//            shooter.shoot();
            shooter.setPower(0.7);

        } else if(gamepad1.left_bumper){
//            shooter.stop();
            shooter.setPower(0);
        }

        if(gamepad1.y){
            controlarServo(0.0);

        }else if (gamepad1.x) {
            controlarServo(0.345);
        }

        if(gamepad1.right_trigger > 0.1){
            intake.intake();
        } else if (gamepad1.left_trigger > 0.1) {
            intake.reverse();
        } else {
            intake.stop();
        }

        driveManual();;
        telemetry.update();
    }

    public void controlarServo(double direcao){
        if(direcao == 0.0){
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(direcao);
        } else if (direcao == 0.345) {
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(direcao);

        }
    }

    private void driveManual() {


        double forward = -gamepad1.left_stick_y;   // frente/trás
        double turn    = gamepad1.right_stick_x;   // giro direita/esquerda

        double leftPower  = Range.clip(forward + turn, -1.0, 1.0);
        double rightPower = Range.clip(forward - turn, -1.0, 1.0);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

    }
}
