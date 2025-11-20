package ftc.team.Java_Is_AllMight.Sensors;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

import ftc.team.allmight.plusultra.teamcode.subsystems.Drive;

public class CameraMight {

    public AprilTagProcessor aprilTagProcessorDosNgc;
    public VisionPortal visionPortalDoNgc;
    public List<AprilTagDetection> detectedTagsDoNGC = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        aprilTagProcessorDosNgc = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessorDosNgc);

        visionPortalDoNgc = builder.build();
    }

    public void update(){
        detectedTagsDoNGC = aprilTagProcessorDosNgc.getDetections();
    }

    public List<AprilTagDetection> getDetectedTagsDoNGC(){
        return detectedTagsDoNGC;
    }

    public AprilTagDetection getTagBySpecificId(int ID){
        for(AprilTagDetection detection : detectedTagsDoNGC){
            if (detection.id == ID){
                return detection;
            }
        }
        return null;
    }

    public void stop(){
        if(visionPortalDoNgc != null){
            visionPortalDoNgc.close();
        }
    }

    public void DisplayDetectionTelemetry(AprilTagDetection detectedId){
        if(detectedId ==  null){
            return;
        }
        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z)); //distancia x,y,z da april tag
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw)); // Pitcvh, Roll, Yaw apartil do Id
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation)); //rANGE(CENTRO DA CAMERA ATE CENTRO DA APRIL TAG // BEARING (DEFLEXÃO) // ELEVATION QUAO ELEVADA ESTA
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }

        // LE ISSO AQUI: BEARING == EQUIVALENTE A TX deslocamento lateral (em graus) — “quanto precisa girar para centralizar a tag”
        // ELEVATION: EQUIVALENTE A TY deslocamento vertical (em graus) — “quanto a tag está acima ou abaixo do centro”
        // RANGE: EQUIVALENTE A TA(CALCULAR 1/RANGE) área aparente       da tag (quanto mais perto, maior o “TA”)

//        double tx = detectedId.ftcPose.bearing;     // equivalente a Limelight tx
//        double ty = detectedId.ftcPose.elevation;   // equivalente a Limelight ty
//        double ta = 1.0 / (detectedId.ftcPose.range * detectedId.ftcPose.range); // aproximação de ta

    }

    // Verifica se um ID está visível
    public boolean isTagVisible(int id) {
        return getTagBySpecificId(id) != null;
    }

    // Distância até uma tag (em metros)
    public double getDistanceToTag(int id) {
        AprilTagDetection tag = getTagBySpecificId(id);
        return tag != null ? tag.ftcPose.range : -1;
    }

    // Distância X (esquerda/direita)
    public double getXOffsetToTag(int id) {
        AprilTagDetection tag = getTagBySpecificId(id);
        return tag != null ? tag.ftcPose.x : 0;
    }

    // Distância Y (frente/trás)
    public double getYOffsetToTag(int id) {
        AprilTagDetection tag = getTagBySpecificId(id);
        return tag != null ? tag.ftcPose.y : 0;
    }

    // Distância Z (altura)
    public double getZOffsetToTag(int id) {
        AprilTagDetection tag = getTagBySpecificId(id);
        return tag != null ? tag.ftcPose.z : 0;
    }

    // Retorna o ID mais próximo (menor distância)
    public AprilTagDetection getClosestTag() {
        AprilTagDetection closest = null;
        double bestRange = Double.MAX_VALUE;

        for (AprilTagDetection tag : detectedTagsDoNGC) {
            if (tag.ftcPose.range < bestRange) {
                bestRange = tag.ftcPose.range;
                closest = tag;
            }
        }
        return closest;
    }

    // Retorna a lista só dos IDs detectados
    public List<Integer> getVisibleTagIDs() {
        List<Integer> ids = new ArrayList<>();
        for (AprilTagDetection tag : detectedTagsDoNGC) {
            ids.add(tag.id);
        }
        return ids;
    }

    // valor de erro para girar em direção ao centro da tag
    public double getTurnErrorToTag(int id) {
        AprilTagDetection tag = getTagBySpecificId(id);
        return tag != null ? tag.ftcPose.bearing : 0;  // bearing = tx
    }

    // Ajuda para avançar até a tag (erro de distância)
    public double getForwardErrorToTag(int id, double desiredRange) {
        AprilTagDetection tag = getTagBySpecificId(id);
        if (tag == null) return 0;
        return tag.ftcPose.range - desiredRange;
    }

    // Converte a pose da tag em coordenadas relativas do robô no campo (útil para localização)
    public double[] getRelativePose(int id) {
        AprilTagDetection tag = getTagBySpecificId(id);
        if (tag == null) return new double[]{0, 0, 0};
        return new double[]{tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.yaw};
    }

    // TX = deslocamento lateral da tag (em graus)
    // equivalente ao "bearing" → quanto precisa girar para centralizar
    public double getTx(int id) {
        AprilTagDetection tag = getTagBySpecificId(id);
        return tag != null ? tag.ftcPose.bearing : 0;
    }

    // TY = deslocamento vertical da tag (em graus)
    // equivalente ao "elevation" → quanto a tag está acima/abaixo do centro
    public double getTy(int id) {
        AprilTagDetection tag = getTagBySpecificId(id);
        return tag != null ? tag.ftcPose.elevation : 0;
    }

    // TA = área aparente da tag (aqui aproximada por 1 / (range²))
    // quanto mais perto a tag está, maior o TA
    public double getTa(int id) {
        AprilTagDetection tag = getTagBySpecificId(id);
        if (tag == null) return 0;

        // A forma mais comum com AprilTag é usar o inverso do range²
        return 1.0 / (tag.ftcPose.range * tag.ftcPose.range);
    }

    // Verifica se a tag está centralizada na câmera
    // tolerance = margem aceitável em graus (ex.: 1.5)
    public boolean isCentered(int id, double tolerance) {
        AprilTagDetection tag = getTagBySpecificId(id);
        if (tag == null) return false;

        double tx = tag.ftcPose.bearing;     // deslocamento lateral
        double ty = tag.ftcPose.elevation;   // deslocamento vertical

        return Math.abs(tx) <= tolerance && Math.abs(ty) <= tolerance;
    }

    public double getYaw(int id) {
        AprilTagDetection tag = getTagBySpecificId(id);
        return tag != null ? tag.ftcPose.yaw : 0;
    }

    public void alignCenterTag(Drive drive, int id, LinearOpMode opMode) {

        double tolerance = 0.5;   // erro aceitável
        double kp = 0.03;         // ganho proporcional
        double minPower = 0.1;   // potência mínima para conseguir girar
        long timeout = System.currentTimeMillis() + 2500; // timeout de segurança

        while (opMode.opModeIsActive() && System.currentTimeMillis() < timeout) {

            double tx = getTx(id);  // bearing da AprilTag

            // Se está alinhado
            if (Math.abs(tx) <= tolerance) {
                drive.stop();
                break;
            }

            // Calcula potência proporcional
            double turnPower = tx * kp;

            // Garante potência mínima
            if (Math.abs(turnPower) < minPower) {
                turnPower = Math.copySign(minPower, turnPower);
            }

            // Gira no lugar
            drive.drive(0, turnPower);
        }

        drive.stop();
    }


}
