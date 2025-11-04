package ftc.team.allmight.plusultra.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import ftc.team.allmight.plusultra.teamcode.subsystems.IntakeSubsytem;
import ftc.team.allmight.plusultra.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Substyem Teste REAL")
public class ShooterTest extends OpMode {

//    ShooterSubsystem shooter = new ShooterSubsystem();
    private DcMotorEx shooter;
    private Servo servo;
    private IntakeSubsytem intake;

    @Override
    public void init() {
//        shooter.init(hardwareMap);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        servo = hardwareMap.get(Servo.class, "servidor");
        intake = new IntakeSubsytem(hardwareMap, telemetry);
        servo.setPosition(0.3);
    }

    @Override
    public void loop() {

        intake.update();

        telemetry.addData("Corrente", shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine(String.valueOf(servo.getPosition()));
        if(gamepad1.right_bumper){
//            shooter.shoot();
            shooter.setPower(1);

        } else if(gamepad1.left_bumper){
//            shooter.stop();
            shooter.setPower(0);
        }

        if(gamepad1.b){
            servo.setPosition(0.8);

        } else if(gamepad1.y){
            servo.setPosition(0);
        } else if (gamepad1.x) {
            servo.setPosition(0.3);
        }

        if(gamepad1.right_trigger > 0.1){
            intake.intake();
        } else {
            intake.stop();
        }


        telemetry.update();
    }
}
