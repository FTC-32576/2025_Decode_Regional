package ftc.team.Java_Is_AllMight.Pathing;

import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.team.Java_Is_AllMight.Config.KalmanFilter;
import ftc.team.Java_Is_AllMight.Sensors.IMUHelper;
import ftc.team.Java_Is_AllMight.Sensors.LimelightHelper;

/**
 * RoadRunnerHelper (TeamLib)
 *
 * - Faz odometria básica por encoders (tank) + IMU yaw
 * - Opcional: usa LimelightHelper para correções de pose quando disponível
 * - Expõe um estado Pose2d (x,y,heading) compatível conceitualmente com RoadRunner
 *
 * Uso:
 * - Instanciar no init do robot/OpMode passando hardwareMap, motores de tração (left,right),
 *   parâmetros de roda/encoders e opcionalmente LimelightHelper e IMUHelper.
 * - Chamar update(dt) periodicamente (dt em segundos).
 */
public class RoadRunnerHelper {

    // ======= parâmetros de robô (tarefas) =======
    public static class DriveParams {
        public final double wheelRadiusMeters; // raio da roda de odometria / pista (m)
        public final double inPerTick; // metros por tick: (2*pi*R / ticksPerRev)
        public final double trackWidth; // distância entre rodas (metros)
        public final int leftEncoderOffset; // se você tiver offsets
        public final int rightEncoderOffset;

        public DriveParams(double wheelRadiusMeters, double inPerTick, double trackWidth, int leftEncOffset, int rightEncOffset) {
            this.wheelRadiusMeters = wheelRadiusMeters;
            this.inPerTick = inPerTick;
            this.trackWidth = trackWidth;
            this.leftEncoderOffset = leftEncOffset;
            this.rightEncoderOffset = rightEncOffset;
        }
    }

    private final HardwareMap hardwareMap;
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private final IMUHelper imu;
    private final LimelightHelper limelight; // opcional (pode ser null)
    private final boolean useLimelight;

    // odometria
    private long lastTimestampNanos = -1;
    private int lastLeftTicks = 0;
    private int lastRightTicks = 0;
    private final DriveParams params;

    // filtro de fusão de pose
    private final KalmanFilter filter;

    // configuração de confiança (variâncias)
    private double odomPosVariance = 0.02 * 0.02; // m^2 (ajustar)
    private double odomHeadingVariance = Math.toRadians(5.0) * Math.toRadians(5.0); // rad^2

    // confiança Limelight (se usado)
    private double limelightPosVariance = 0.05 * 0.05; // m^2 (ajustar)
    private double limelightHeadingVariance = Math.toRadians(8.0) * Math.toRadians(8.0);

    /**
     * Construtor
     * @param hardwareMap HardwareMap
     * @param leftMotorName nome do motor/encoder esquerdo
     * @param rightMotorName nome do motor/encoder direito
     * @param params parâmetros do drive/encoders
     * @param imuHelper IMUHelper (obrigatório)
     * @param limelightHelper LimelightHelper (opcional - pode ser null)
     * @param startPose pose inicial (metros / radians)
     */
    public RoadRunnerHelper(HardwareMap hardwareMap,
                            String leftMotorName,
                            String rightMotorName,
                            DriveParams params,
                            IMUHelper imuHelper,
                            LimelightHelper limelightHelper,
                            Pose2d startPose) {

        this.hardwareMap = hardwareMap;
        this.leftMotor = hardwareMap.get(DcMotor.class, leftMotorName);
        this.rightMotor = hardwareMap.get(DcMotor.class, rightMotorName);
        this.leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.params = params;
        this.imu = imuHelper;
        this.limelight = limelightHelper;
        this.useLimelight = (limelightHelper != null);

        // inicializa odometria
        this.lastLeftTicks = leftMotor.getCurrentPosition();
        this.lastRightTicks = rightMotor.getCurrentPosition();
        this.lastTimestampNanos = System.nanoTime();

        // inicializa KF
        double initCovPos = 0.1;
        double initCovHeading = Math.toRadians(10.0) * Math.toRadians(10.0);
        this.filter = new KalmanFilter(startPose.position.x, startPose.position.y, startPose.heading.toDouble(),
                initCovPos, initCovHeading,
                /*qPos*/ odomPosVariance, /*qHeading*/ odomHeadingVariance);
    }

    /**
     * Atualiza odometria + filtro. Deve ser chamado em loop (periodicamente).
     * @param dtSeconds intervalo em segundos desde última chamada (se preferir). Se dtSeconds <= 0, será calculado internamente.
     */
    public void update(double dtSeconds) {
        long now = System.nanoTime();
        if (lastTimestampNanos <= 0) {
            lastTimestampNanos = now;
        }
        double dt = dtSeconds;
        if (dtSeconds <= 0) {
            dt = (now - lastTimestampNanos) / 1e9;
        }
        lastTimestampNanos = now;

        // leitura encoders
        int curLeft = leftMotor.getCurrentPosition();
        int curRight = rightMotor.getCurrentPosition();

        int dLeft = curLeft - lastLeftTicks;
        int dRight = curRight - lastRightTicks;

        lastLeftTicks = curLeft;
        lastRightTicks = curRight;

        // converte para metros (usando inPerTick)
        double leftMeters = dLeft * params.inPerTick;
        double rightMeters = dRight * params.inPerTick;

        // delta robot-frame
        double deltaForward = (leftMeters + rightMeters) / 2.0;
        double deltaHeading = (rightMeters - leftMeters) / params.trackWidth; // radianos aproximados small-angle

        // converte delta em campo-frame usando heading atual do filtro
        double curHeading = filter.getHeading();
        double cosH = Math.cos(curHeading);
        double sinH = Math.sin(curHeading);

        // movimento no campo: rotate (deltaForward, 0) pelo heading atual
        double deltaX_field = deltaForward * cosH;
        double deltaY_field = deltaForward * sinH;

        // predicao do filtro com odometria
        filter.predict(deltaX_field, deltaY_field, deltaHeading);

        // se estiver usando limelight, tenta corrigir quando for válido e recente
        if (useLimelight && limelight.isConnected()) {
            // obtém resultado e usa botpose (MT2 preferida)
            try {
                // LimelightHelper expõe getEstimatedFieldPosition() retornando Pose3D
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D llPose3d = limelight.getEstimatedFieldPosition();
                if (llPose3d != null) {
                    // converte para Pose2d (RoadRunner)
                    double lx = llPose3d.getPosition().x; // metros
                    double ly = llPose3d.getPosition().y;
                    double lHeadingDeg = llPose3d.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
                    double lHeading = Math.toRadians(lHeadingDeg);

                    // Estimativa de covariância: você pode melhorar isso por confiança/estabilidade do limelight
                    double rPos = limelightPosVariance;
                    double rHeading = limelightHeadingVariance;

                    // Corrige o filtro com a pose da Limelight
                    filter.correct(lx, ly, lHeading, rPos, rHeading);
                }
            } catch (Exception e) {
                // se algo falhar (nulls), apenas ignore a correção
            }
        }


    }

    /** Retorna a pose estimada (Pose2d) */
    public Pose2d getPose() {
        return new Pose2d(filter.getX(), filter.getY(), filter.getHeading());
    }

    /** Define pose estimada (ex.: zerar pose no início) */
    public void setPose(Pose2d pose) {
        filter.setState(pose.position.x, pose.position.y, pose.heading.toDouble());
    }

    /** Reset simples (não reseta encoders) */
    public void resetPose(Pose2d pose) {
        setPose(pose);
    }

    public void setOdomCovariances(double posVar, double headingVar) {
        this.odomPosVariance = posVar;
        this.odomHeadingVariance = headingVar;
    }

    public void setLimelightCovariances(double posVar, double headingVar) {
        this.limelightPosVariance = posVar;
        this.limelightHeadingVariance = headingVar;
    }
}
