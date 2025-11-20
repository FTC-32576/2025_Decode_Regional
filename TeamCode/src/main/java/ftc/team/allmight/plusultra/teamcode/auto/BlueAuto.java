package ftc.team.allmight.plusultra.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import ftc.team.Java_Is_AllMight.Config.PIDConfig;
import ftc.team.Java_Is_AllMight.Config.PIDController;
import ftc.team.Java_Is_AllMight.Sensors.CameraMight;
import ftc.team.Java_Is_AllMight.Utils.Alliance;
import ftc.team.allmight.plusultra.teamcode.subsystems.Drive;
import ftc.team.Java_Is_AllMight.Sensors.IMUMight;
import ftc.team.allmight.plusultra.teamcode.subsystems.IntakeSubsytem;
import ftc.team.allmight.plusultra.teamcode.utils.AprilTagPatternUtil;

@Autonomous(name = "Blue - PERTO Simples", group = "A")
public class BlueAuto extends LinearOpMode {

    private AprilTagPatternUtil patternUtil;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private AprilTagProcessor aprilTagProcessor;

    CameraMight cameraMight = new CameraMight();

    IntakeSubsytem intake;

    // PID do shooter
    private PIDConfig pidShooterSettings = new PIDConfig(
            0.00435,  // P
            0.0,
            0.0
    );
    private PIDController pidShooter = new PIDController(pidShooterSettings);

    // Regressão distância → velocidade alvo
    private static final double M = 380.8306010928962;
    private static final double B = 554.551912568306;
    private static final double ALPHA = 0.25;
    private double filteredTarget = 0;

    // estabilidade (ticks)
    private static final double STABLE_THRESHOLD = 50.0;

    public enum shootNumber{
        START(),
        DEPOIS()
    }


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
        drive.defineAlliance(Alliance.BLUE);

        telemetry.addLine("Pronto");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        servo.setPosition(0.0); //servo pro 0

        // Shooter começa acelerando para uma velocidade segura enquanto ainda não tem tag
        double preBoost = 0.8; // Ajuste conforme seu robô
        shooter.setPower(preBoost);
        filteredTarget = preBoost;  // Começa o filtro daqui




        //        for (int i = 0; i < 10 && opModeIsActive(); i++) {
        //            double erro = adjustShooterPower(shooter, Alliance.RED);
        //            if (erro < STABLE_THRESHOLD) break;
        //        }



        drive.moveSmooth(-135, 1); // ir pra ponta do triangulo grande


        shootar(5, 0.45678901, 0, shooter, servo, Alliance.RED, shootNumber.START); // shootar 3 bolas + 1 por segurança

        drive.turnIMU(50, 0.35, 1, telemetry); // girar pra ESQUERDA(TA INVERTIDA LEMBRA DISSO), ficar praticamente 90 gruas

        drive.moveSmooth(-46, 0.95); // ir pra tras pra linha de bolas
        sleep(35);
        drive.turnIMU(-90, 0.39, 6, telemetry); // girar pra linha de bolas PPG
        sleep(35); // espera antes de andar pra ser preciso
        drive.moveSmooth(48.5, 0.94); // andar ate  frente

        intake.intake(); // INTAKEEEEEEEEEE
        drive.moveSmoothStart(53, 0.22); // coletar

        while (opModeIsActive() && (intake.intakeIsBusy() || drive.moveSmoothIsBusy())) {
            drive.moveSmoothUpdate();   // mantém o drive andando
            intake.update();            // mantém o intake rodando
        }

        drive.stop();
        intake.stop();
        intake.update();

        drive.moveSmooth(-100, 1);
        drive.turnIMU(45, 0.7, 1, telemetry);
        // prepara shooter para nova posição
        filteredTarget = 0;

        shootar(5, 0.45678901, 0, shooter, servo, Alliance.RED, shootNumber.DEPOIS);

        // ENCERRA
        drive.stop();

        while (opModeIsActive()) {
            telemetry.addData("IMU", imu.getYaw());
            telemetry.update();
        }

    }

    public void shootar(int shots, double posX, double posY, DcMotorEx shooter, Servo servo, Alliance alliance, shootNumber shoot) {

        intake.intake();

        for (int i = 0; i < shots && opModeIsActive(); i++) {
            if (i > 1) intake.update();

            if (i == 0){
                if (shoot == shoot.START){
                    // aguarda shooter estabilizar (com timeout de segurança)
                    long timeout = System.currentTimeMillis() + 2500; // 2.5s máximo para estabilizar
                    while (opModeIsActive() && System.currentTimeMillis() < timeout) {
                        double erro = adjustShooterPower(shooter, alliance);
                        if (erro < STABLE_THRESHOLD) break;
                    }
                } else{
                    // aguarda shooter estabilizar (com timeout de segurança)
                    long timeout = System.currentTimeMillis() + 3250; // 2.15s máximo para estabilizar
                    while (opModeIsActive() && System.currentTimeMillis() < timeout) {
                        double erro = adjustShooterPower(shooter, alliance);
                        if (erro < STABLE_THRESHOLD) break;
                    }
                    intake.update();
                }

            } else{
                // aguarda shooter estabilizar (com timeout de segurança)
                long timeout = System.currentTimeMillis() + 1600; // 2s máximo para estabilizar
                while (opModeIsActive() && System.currentTimeMillis() < timeout) {
                    double erro = adjustShooterPower(shooter, alliance);
                    if (erro < STABLE_THRESHOLD) break;
                }
            }


            // dispara
            servo.setPosition(posX);
            sleep(350);
            servo.setPosition(posY);

            telemetry.addData("LOOP", i);
            telemetry.update();
            sleep(850);
        }
        shooter.setPower(0);
        intake.stop();
        intake.update();
    }
    private AprilTagDetection getAllianceTag(List<AprilTagDetection> detections, int allianceTagID) {
        if (detections == null) return null;
        for (AprilTagDetection tag : detections) {
            if (tag.id == allianceTagID) {
                return tag;
            }
        }
        return null;
    }

    private double adjustShooterPower(DcMotorEx shooter, Alliance alliance) {


        boolean firstTagFound = false;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagDetection tag = null;

        if (detections != null && !detections.isEmpty()) {
            tag = getAllianceTag(detections, alliance.getTagID());
        }

        if (tag == null) {
            // Mantém o shooter rodando com o último target
            double atual = shooter.getVelocity();
            double pidOut = pidShooter.calculate(filteredTarget, atual);
            shooter.setPower(Range.clip(pidOut, -1, 1));

            telemetry.addLine("SEM TAG → usando target anterior");
            telemetry.update();

            return Double.POSITIVE_INFINITY;
        }

        // Se achou tag pela primeira vez
        if (!firstTagFound) {
            filteredTarget = M * tag.ftcPose.range + B;
            firstTagFound = true;
        }

        double dist = tag.ftcPose.range;

        double targetTicks = M * dist + B;
        targetTicks = Range.clip(targetTicks, 0, 2000);

        if (filteredTarget == 0) filteredTarget = targetTicks;
        else filteredTarget = ALPHA * targetTicks + (1 - ALPHA) * filteredTarget;

        double atual = shooter.getVelocity();
        double pidOut = pidShooter.calculate(filteredTarget, atual);

        shooter.setPower(Range.clip(pidOut, -1, 1));

        return Math.abs(atual - filteredTarget);
    }
}