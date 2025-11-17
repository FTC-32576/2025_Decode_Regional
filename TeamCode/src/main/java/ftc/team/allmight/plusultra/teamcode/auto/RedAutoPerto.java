package ftc.team.allmight.plusultra.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import ftc.team.Java_Is_AllMight.Sensors.CameraMight;
import ftc.team.allmight.plusultra.teamcode.subsystems.Drive;
import ftc.team.Java_Is_AllMight.Sensors.IMUMight;
import ftc.team.allmight.plusultra.teamcode.subsystems.IntakeSubsytem;
import ftc.team.allmight.plusultra.teamcode.subsystems.ServoSubsystem;
import ftc.team.allmight.plusultra.teamcode.utils.AprilTagPatternUtil;

@Autonomous(name = "AUTO - PERTO Simples", group = "A")
public class RedAutoPerto extends LinearOpMode {

    private AprilTagPatternUtil patternUtil;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private AprilTagProcessor aprilTagProcessor;

    CameraMight cameraMight = new CameraMight();

    IntakeSubsytem intake;


    @Override
    public void runOpMode() throws InterruptedException {

        intake = new IntakeSubsytem(hardwareMap, telemetry);
        // IMU
        IMUMight imu = new IMUMight(hardwareMap, "imu", RevHubOrientationOnRobot.UsbFacingDirection.UP, RevHubOrientationOnRobot.LogoFacingDirection.RIGHT);
        imu.resetYaw();

        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients());
        cameraMight.init(hardwareMap, telemetry);
        patternUtil = new AprilTagPatternUtil(cameraMight.visionPortalDoNgc, cameraMight.aprilTagProcessorDosNgc);

        aprilTagProcessor = cameraMight.aprilTagProcessorDosNgc;

        Servo servo = hardwareMap.get(Servo.class, "servidor");

        // DRIVE
        Drive drive = new Drive(hardwareMap, imu);

        telemetry.addLine("Pronto");
        telemetry.update();

        waitForStart();


        if (isStopRequested()) return;

        shooter.setPower(0.81927046193961946204986);

        drive.moveSmooth(-140, 1);

        cameraMight.alignCenterTag(drive, 24,this);

        shootar(4,0.345, 0, shooter, servo);

        drive.turnIMU(50, 0.45, 5.5, telemetry);



        AprilTagPatternUtil.Pattern finalPattern = null;

        telemetry.addLine("Procurando pattern...");
        telemetry.update();

        while (opModeIsActive() && finalPattern == null) {

            AprilTagPatternUtil.Pattern p = patternUtil.detectPattern(telemetry);

            if (p != null) {
                finalPattern = p;   // trava o padrão – nunca mais atualiza
            }

            sleep(80); // evita spam no sistema
        }

        telemetry.addData("Pattern FINAL", finalPattern);
        telemetry.update();

        // -----------------------------
        // AÇÃO PARA CADA PADRÃO
        // -----------------------------


        switch (finalPattern){
            case PPG:
                telemetry.addLine("PPG detectado");
                telemetry.update();
                drive.moveSmooth(-50, 0.6);
                drive.turnIMU(-78, 0.55, 5.5, telemetry);
                drive.moveSmooth(58, 0.9);

                intake.intake();
                drive.moveSmoothStart(55, 0.15);

                while (opModeIsActive() && (intake.intakeIsBusy() || drive.moveSmoothIsBusy())) {
                    drive.moveSmoothUpdate();   // mantém o drive andando
                    intake.update();            // mantém o intake rodando
                }

                drive.stop();
                intake.stop();
                intake.update();

                drive.moveSmooth(-100, 0.6);
                drive.turnIMU(35, 0.6, 5.5,telemetry);
                sleep(200);
                shootar(4,0.345, 0, shooter, servo);

            case PGP:
                telemetry.addLine("PGP detectado → movendo 20 cm pra frente");
                telemetry.update();
//                drive.moveSmooth(20, 0.6);
            case GPP:
                telemetry.addLine("GPP detectado → girando 90°");
                telemetry.update();
//                drive.turnIMU(90, 0.5, 5, telemetry);
            default:
                telemetry.addLine("Nenhum pattern válido detectado.");
                telemetry.update();
        }

        // ENCERRA
        drive.stop();

        while(opModeIsActive()) {
            telemetry.addData("IMU", imu.getYaw());
            telemetry.update();
        }


    }

    public void shootar(int shots, double posX, double posY, DcMotorEx shooter, Servo servo){
//        shooter.setPower(0.8);
        intake.intake();
        for (int i =0; i <= shots; i++){
            if (i > 1) intake.update();

            servo.setPosition(posX);
            telemetry.addData("Shooter", shooter.getVelocity(AngleUnit.DEGREES));
            telemetry.update();
            sleep(400);

            servo.setPosition(posY);
            sleep(1500);
        }

        intake.stop();
        intake.update();

    }

    private AprilTagDetection getAllianceTag(List<AprilTagDetection> detections, int allianceTagID) {
        for (AprilTagDetection tag : detections) {
            if (tag.id == allianceTagID) {
                return tag;
            }
        }
        return null;
    }


}
