package ftc.team.Java_Is_AllMight.Sensors;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import ftc.team.Java_Is_AllMight.Control.PIDConfig;
import ftc.team.Java_Is_AllMight.Control.PIDController;


public class LimelightHelper {


    private final Limelight3A limelight;
    private final IMUHelper imu;
    private final PIDConfig pidDistance;
    private final PIDConfig pidAngle;

    private double lastDistanceError, distanceIntegral, lastAngleError, angleIntegral = 0;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private PIDController distancePID;
    private PIDController  yawPID;

    public LimelightHelper(HardwareMap hardwareMap, String cameraName,
                           String imuName,
                           RevHubOrientationOnRobot.UsbFacingDirection usbDir,
                           RevHubOrientationOnRobot.LogoFacingDirection logoDir,
                           PIDConfig pidAngle, PIDConfig pidDistance) {

        this.limelight = hardwareMap.get(Limelight3A.class, cameraName);
        this.imu = new IMUHelper(hardwareMap, imuName, usbDir, logoDir);

        this.pidAngle = pidAngle;
        this.pidDistance = pidAngle;
    }


    // ==========================
    // MÉTODOS PARA OBTER INFORMAÇÕES
    // ==========================

    /** Retorna o deslocamento horizontal do alvo (tx) */
    public double getTx() {
        return limelight.getLatestResult().getTx();
    }

    /** Retorna o deslocamento vertical do alvo (ty) */
    public double getTy() {
        return limelight.getLatestResult().getTy();
    }

    /** Retorna a área do alvo detectada (ta) */
    public double getTa() {
        return limelight.getLatestResult().getTa();
    }

    /** Verifica se existe alvo válido */
    public boolean isTargetValid() {
        return limelight.getLatestResult().isValid();
    }

    /** Retorna a pose 3D do robô */
    public Pose3D getBotPoseMT1() {
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()){
            return r.getBotpose();
        }
        return null;
    }

    /** Retorna a pose MT2 do robô (com yaw atualizado) */
    public Pose3D getBotPoseMT2() {
        updateRobotOrientation();
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()){
            return r.getBotpose_MT2();
        }
        return null;

    }
    public List<LLResultTypes.FiducialResult> getFiducials() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return null;
    }

    public double getDistanceToFiducial(LLResultTypes.FiducialResult fiducial) {
        if (fiducial != null) {

            Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
            return Math.sqrt(
                    robotPose.getPosition().x    * robotPose.getPosition().x +
                            robotPose.getPosition().y * robotPose.getPosition().y +
                            robotPose.getPosition().z * robotPose.getPosition().z
            );
        }
        return -1;
    }

    public double getYawToFiducial(LLResultTypes.FiducialResult fiducial) {
        if (fiducial != null) {
            Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
            return Math.toDegrees(Math.atan2(robotPose.getPosition().y, robotPose.getPosition().x));
        }
        return 0;
    }

    public double getPitchToFiducial(LLResultTypes.FiducialResult fiducial) {
        if (fiducial != null) {
            Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
            double horizontalDist = Math.sqrt(robotPose.getPosition().x*robotPose.getPosition().x + robotPose.getPosition().y*robotPose.getPosition().y);
            return Math.toDegrees(Math.atan2(robotPose.getPosition().z, horizontalDist));
        }
        return 0;
    }

    public Pose3D getEstimatedFieldPosition() {
        updateRobotOrientation();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D mt2 = result.getBotpose_MT2();
            if (mt2 != null) return mt2;
            Pose3D mt1 = result.getBotpose();
            if (mt1 != null) return mt1;
        }
        return null;
    }

    // ==========================
    // TELEMETRIA
    // ==========================

    public void telemetryFull(Telemetry telemetry) {
        Pose3D fieldPose = getEstimatedFieldPosition();
        if (fieldPose != null) {
            telemetry.addData("Robô (Campo)", "X: %.2f Y: %.2f Z: %.2f", fieldPose.getPosition().x, fieldPose.getPosition().y, fieldPose.getPosition().z);
        }

        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult f : fiducials) {
                double dist = getDistanceToFiducial(f);
                double yaw = getYawToFiducial(f);
                double pitch = getPitchToFiducial(f);
                telemetry.addData("Fiducial " + f.getFiducialId(), String.format("Dist: %.2fm Yaw: %.1f° Pitch: %.1f°", dist, yaw, pitch));
            }
        }
    }

    // ==========================
    // MOVIMENTO PARA POSIÇÃO
    // ==========================

    /**
     * Calcula a direção e velocidade para ir até a posição (xTarget, yTarget) no campo
     * @param xTarget coordenada X desejada
     * @param yTarget coordenada Y desejada
     * @return array [vX, vY, vRot] para mover o robô
     */
    public double[] calculateMovementToPosition(double xTarget, double yTarget, double dtSeconds) {
        Pose3D currentPose = getEstimatedFieldPosition();
        if (currentPose == null) return new double[]{0,0,0};

        double dx = xTarget - currentPose.getPosition().x;
        double dy = yTarget - currentPose.getPosition().y;

        double distance = Math.sqrt(dx*dx + dy*dy);
        double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));

        double yawError = imu.normalizeAngle(angleToTarget - imu.getYaw());

        // PID distância
        distanceIntegral += distance * dtSeconds;
        double distanceDerivative = (distance - lastDistanceError) / dtSeconds;
        lastDistanceError = distance;
        double vForward = pidDistance.kP * distance + pidDistance.kI * distanceIntegral + pidDistance.kD * distanceDerivative;

        // PID rotação
        angleIntegral += yawError * dtSeconds;
        double angleDerivative = (yawError - lastAngleError) / dtSeconds;
        lastAngleError = yawError;
        double vRot = pidAngle.kP * yawError + pidAngle.kI * angleIntegral + pidAngle.kD * angleDerivative;

        // Limitar valores
        vForward = Math.min(vForward, 1);
        vRot = Math.max(Math.min(vRot, 1), -1);

        return new double[]{vForward, 0, vRot}; // [frente, strafe (0 para tank), rotação]
    }

    /** Retorna o tempo desde a última atualização (ms) */
    public long getTimeSinceLastUpdate() {
        return limelight.getTimeSinceLastUpdate();
    }

    /** Verifica se o Limelight está conectado */
    public boolean isConnected() {
        return limelight.isConnected();
    }

    // ==========================
    // MÉTODOS PARA PYTHON SNAP
    // ==========================

    /** Atualiza os inputs do Python SnapScript (8 valores) */
    public boolean updatePythonInputs(double i1, double i2, double i3, double i4,
                                      double i5, double i6, double i7, double i8) {
        return limelight.updatePythonInputs(i1,i2,i3,i4,i5,i6,i7,i8);
    }

    /** Atualiza os inputs do Python SnapScript usando array */
    public boolean updatePythonInputs(double[] inputs) {
        return limelight.updatePythonInputs(inputs);
    }

    // ==========================
    // MÉTODOS DE PIPELINE
    // ==========================

    /** Muda para o pipeline desejado pelo índice */
    public boolean switchPipeline(int index) {
        return limelight.pipelineSwitch(index);
    }

    /** Recarrega o pipeline atual */
    public boolean reloadPipeline() {
        return limelight.reloadPipeline();
    }

    /** Captura um snapshot */
    public boolean captureSnapshot(String name) {
        return limelight.captureSnapshot(name);
    }

    /** Deleta um snapshot */
    public boolean deleteSnapshot(String name) {
        return limelight.deleteSnapshot(name);
    }

    // ==========================
    // MÉTODOS DE CAMPO / ORIENTAÇÃO
    // ==========================

    /** Atualiza a orientação do robô para MegaTag2 */
    private void updateRobotOrientation() {
        double yaw = imu.getYaw();
        limelight.updateRobotOrientation(yaw);
    }

    /** Faz upload de um mapa de campo */
    public boolean uploadFieldmap(LLFieldMap map, Integer index) {
        return limelight.uploadFieldmap(map, index);
    }

    // ==========================
    // CONTROLE DE POLLING
    // ==========================

    public void start() {
        limelight.start();
    }

    public void pause() {
        limelight.pause();
    }

    public void stop() {
        limelight.stop();
    }

    public boolean isRunning() {
        return limelight.isRunning();
    }

    /** Define a taxa de atualização em Hz */
    public void setPollRateHz(int hz) {
        limelight.setPollRateHz(hz);
    }

    // ==========================
    // MÉTODOS MISC
    // ==========================

    /** Retorna a latência da captura (ms) */
    public double getCaptureLatency() {
        return limelight.getLatestResult().getCaptureLatency();
    }

    /** Retorna a latência do processamento do alvo (ms) */
    public double getTargetingLatency() {
        return limelight.getLatestResult().getTargetingLatency();
    }

    /** Retorna a latência do parse do resultado (ms) */
    public double getParseLatency() {
        return limelight.getLatestResult().getParseLatency();
    }

    public Integer getClosestFiducialID(){
        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if(fiducials == null || fiducials.isEmpty()) return null;

        // Escolhe a fiducial mais próxima (menor distância)
        LLResultTypes.FiducialResult closest = fiducials.get(0);
        double minDist = getDistanceToFiducial(closest);
        for(LLResultTypes.FiducialResult f : fiducials){
            double dist = getDistanceToFiducial(f);
            if(dist < minDist){
                minDist = dist;
                closest = f;
            }
        }
        return closest.getFiducialId();
    }


    public void setupMovement(DcMotor leftMotor, DcMotor rightMotor, PIDConfig distanceConfig, PIDConfig yawConfig){
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.distancePID = new PIDController(distanceConfig);
        this.yawPID = new PIDController(yawConfig);
    }

    /**
     * Move o robô até a AprilTag, parando na distância desejada
     *
     * @param desiredDistance  Distância desejada até a tag (metros)
     * @param maxPower         Potência máxima para os motores
     * @param telemetry        Telemetry para debug
     * @return true se alcançou a distância
     */
    public boolean moveToFiducialByIdTank(int targetID, double desiredDistance, double maxPower, Telemetry telemetry){

        if(leftMotor == null || rightMotor == null || distancePID == null || yawPID == null) return false;

        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if(fiducials == null || fiducials.isEmpty()) return false;

        // Pega a fiducial mais próxima

        LLResultTypes.FiducialResult target = null;
        for(LLResultTypes.FiducialResult f : fiducials){
            if(f.getFiducialId() == targetID){
                target = f;

                break;
            }
        }

        // Distância atual
        double distance = getDistanceToFiducial(target);
        double distanceError = distance - desiredDistance;

        // Se dentro da tolerância, parar motores
        if(Math.abs(distanceError) < 0.05){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return true;
        }

        // Comando PID para distância
        double forwardPower = distancePID.calculate(desiredDistance,distanceError);
        forwardPower = Math.max(-maxPower, Math.min(maxPower, forwardPower));

        // Correção de yaw
        double yawToTag = getYawToFiducial(target);
        double yawCorrection = yawPID.calculate(0, yawToTag);

        // Ajuste motores
        double leftPower = forwardPower + yawCorrection;
        double rightPower = forwardPower - yawCorrection;

        leftPower = Math.max(-maxPower, Math.min(maxPower, leftPower));
        rightPower = Math.max(-maxPower, Math.min(maxPower, rightPower));

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // Telemetry
        if(telemetry != null){
            telemetry.addData("Fiducial ID", target.getFiducialId());
            telemetry.addData("Dist Error", distanceError);
            telemetry.addData("Forward Power", forwardPower);
            telemetry.addData("Yaw To Tag", yawToTag);
            telemetry.addData("Yaw Correction", yawCorrection);
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }

        return false;
    }

    public boolean moveToFiducialByIdMecanum(int targetID,DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, double desiredDistance, double maxPower, Telemetry telemetry){

        if(frontLeft == null || frontRight == null || backLeft == null || backRight == null || distancePID == null || yawPID == null)
            return false;

        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if(fiducials == null || fiducials.isEmpty()) return false;

        // Procura fiducial com o ID desejado
        LLResultTypes.FiducialResult target = null;
        for(LLResultTypes.FiducialResult f : fiducials){
            if(f.getFiducialId() == targetID){
                target = f;
                break;
            }
        }
        if(target == null) return false;

        // Distância e ângulo para a fiducial
        double distance = getDistanceToFiducial(target);
        double distanceError = distance - desiredDistance;
        double yawToTag = getYawToFiducial(target);

        // Se dentro da tolerância, parar motores
        if(Math.abs(distanceError) < 0.05){
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return true;
        }

        // PID para distância
        double forwardPower = distancePID.calculate(desiredDistance, distanceError);
        forwardPower = clamp(forwardPower, -maxPower, maxPower);

        // PID para yaw
        double yawCorrection = yawPID.calculate(0, yawToTag);

        // Movimentação lateral (strafe) baseada no ângulo da fiducial
        // Transformamos o vetor polar (distance, yawToTag) em componentes X e Y
        double strafePower = forwardPower * Math.sin(Math.toRadians(yawToTag));
        double forwardComponent = forwardPower * Math.cos(Math.toRadians(yawToTag));

        // Ajuste de cada motor para mecanum
        double fl = forwardComponent + strafePower + yawCorrection;
        double fr = forwardComponent - strafePower - yawCorrection;
        double bl = forwardComponent - strafePower + yawCorrection;
        double br = forwardComponent + strafePower - yawCorrection;

        // Limita cada motor
        fl = clamp(fl, -maxPower, maxPower);
        fr = clamp(fr, -maxPower, maxPower);
        bl = clamp(bl, -maxPower, maxPower);
        br = clamp(br, -maxPower, maxPower);

        // Define potência
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        // Telemetria
        if(telemetry != null){
            telemetry.addData("Fiducial ID", target.getFiducialId());
            telemetry.addData("Distance Error", distanceError);
            telemetry.addData("Forward", forwardComponent);
            telemetry.addData("Strafe", strafePower);
            telemetry.addData("Yaw Correction", yawCorrection);
            telemetry.addData("FL", fl);
            telemetry.addData("FR", fr);
            telemetry.addData("BL", bl);
            telemetry.addData("BR", br);
            telemetry.update();
        }

        return false;
    }

    private double clamp(double value, double min, double max){
        return Math.max(min, Math.min(max, value));
    }



}