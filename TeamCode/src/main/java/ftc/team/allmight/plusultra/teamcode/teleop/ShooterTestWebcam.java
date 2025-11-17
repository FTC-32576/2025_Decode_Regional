    package ftc.team.allmight.plusultra.teamcode.teleop;

    import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.util.Range;

    import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

    import java.util.List;

    import ftc.team.Java_Is_AllMight.Config.PIDConfig;
    import ftc.team.Java_Is_AllMight.Config.PIDController;
    import ftc.team.Java_Is_AllMight.Sensors.IMUMight;
    import ftc.team.allmight.plusultra.teamcode.subsystems.Drive;
    import ftc.team.allmight.plusultra.teamcode.subsystems.IntakeSubsytem;
    import ftc.team.allmight.plusultra.teamcode.subsystems.ServoSubsystem;
    import ftc.team.Java_Is_AllMight.Sensors.CameraMight;
    import ftc.team.Java_Is_AllMight.Utils.Alliance;

    @TeleOp(name = "ShooterTest Webcam PID REAL FINAL")
    public class ShooterTestWebcam extends OpMode {

        private DcMotorEx shooter;
        private IntakeSubsytem intake;
        private ServoSubsystem servo;
        private Drive drive;

        private CameraMight webcam;
        private Alliance alliance = Alliance.RED;

        // ===== PID =====
        private PIDConfig pidShooterSettings = new PIDConfig(
                0.00435,  // P
                0.0,       // I
                0.0        // D
        );
        private PIDController pidShooter = new PIDController(pidShooterSettings);

        // ===== LINEAR =====
        private static final double M = 330.8306010928962;
        private static final double B = 543.551912568306;
        private double filteredTarget = 0.0;
        private static final double ALPHA = 0.25;

        // LIMITES
        private static final double TARGET_MIN = 0;
        private static final double TARGET_MAX = 2000;

        // Shooter estado
        private boolean shooterOn = false;

        // ===== AUTO SHOOT =====
        private static final int TOLERANCIA = 20;
        private static final int STABLE_REQUIRED = 4;

        private int stableCounter = 0;
        private boolean servoBusy = false;
        private long lastShotTime = 0;


        @Override
        public void init() {

            shooter = hardwareMap.get(DcMotorEx.class, "shooter");
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            intake = new IntakeSubsytem(hardwareMap, telemetry);
            servo = new ServoSubsystem(hardwareMap);
            drive = new Drive(hardwareMap, new IMUMight(hardwareMap, "imu", RevHubOrientationOnRobot.UsbFacingDirection.UP, RevHubOrientationOnRobot.LogoFacingDirection.RIGHT));

            webcam = new CameraMight();
            webcam.init(hardwareMap, telemetry);

            telemetry.addLine("iniciado!");
        }



        @Override
        public void init_loop(){
            if(gamepad2.x){
                alliance = Alliance.BLUE;
            } else if(gamepad2.b){
                alliance = Alliance.RED;
            }

            telemetry.addData("Selecionar Aliança", "X = BLUE | B = RED");
            telemetry.addData("Aliança atual", alliance);
            telemetry.update();
        }

        @Override
        public void loop() {

            // ---------------- STOP ----------------
            if (gamepad2.back) {
                intake.stop();
                shooter.setPower(0);
                servo.setPosition(0.0);
            }
            else {


                if (gamepad2.a) intake.intake();
                else if (gamepad2.b) intake.reverse();
                else intake.stop();
            }

            intake.update();

            // ---------------- SHOOTER ------------
            // ----
            boolean webcamMode = gamepad2.right_bumper;
            double rt = gamepad2.right_trigger;

            if (webcamMode) {

                // ======= WEBCAM + PID =======
                List<AprilTagDetection> detections = webcam.aprilTagProcessorDosNgc.getDetections();
                AprilTagDetection tag = findAllianceTag(detections, alliance.getTagID());

                if (tag != null) {

                    double distancia = tag.ftcPose.range;

                    // regressão
                    double targetTicks = M * distancia + B;
                    targetTicks = Range.clip(targetTicks, TARGET_MIN, TARGET_MAX);

                    // filtro
                    if (filteredTarget == 0) filteredTarget = targetTicks;
                    else filteredTarget = ALPHA * targetTicks + (1 - ALPHA) * filteredTarget;

                    // PID
                    double atual = shooter.getVelocity();
                    double pidOut = pidShooter.calculate(filteredTarget, atual);
                    double power = Range.clip(pidOut, -1, 1);

                    shooter.setPower(power);

                    telemetry.addLine("MODE: WEBCAM");
                    telemetry.addData("dist", distancia);
                    telemetry.addData("target", filteredTarget);
                    telemetry.addData("vel atual", atual);
                }
                else {
                    shooter.setPower(0);
                    telemetry.addLine("WEBCAM MODE - NO TAG");
                }
            }
            else {
               //manual
                shooter.setPower(Range.clip(rt, 0, 0.8375));

                telemetry.addLine("MODE: MANUAL");
                telemetry.addData("power", rt);
            }


            // ---------------- SERVO ----------------
            if (gamepad2.x) servo.setPosition(0.345);
            if (gamepad2.y) servo.setPosition(0.0);

            // ---------------- DRIVE ----------------
            drive.drive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.update();
        }

        private AprilTagDetection findAllianceTag(List<AprilTagDetection> detections, int tagId) {
            if (detections == null) return null;
            for (AprilTagDetection t : detections) {
                if (t.id == tagId) return t;
            }
            return null;
        }
    }
