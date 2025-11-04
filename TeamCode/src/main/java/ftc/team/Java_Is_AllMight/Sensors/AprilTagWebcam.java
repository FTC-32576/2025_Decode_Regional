package ftc.team.Java_Is_AllMight.Sensors;

import android.util.Size;

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

public class AprilTagWebcam {

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

        // LE ISSO AQUI: BEARING ==EQUIVALENTE A TX deslocamento lateral (em graus) — “quanto precisa girar para centralizar a tag”
        // ELEVATION: EQUIVALENTE A TY deslocamento vertical (em graus) — “quanto a tag está acima ou abaixo do centro”
        // RANGE: EQUIVALENTE A TA(CALCULAR 1/RANGE) área aparente da tag (quanto mais perto, maior o “TA”)

//        double tx = detectedId.ftcPose.bearing;     // equivalente a Limelight tx
//        double ty = detectedId.ftcPose.elevation;   // equivalente a Limelight ty
//        double ta = 1.0 / (detectedId.ftcPose.range * detectedId.ftcPose.range); // aproximação de ta



    }
}
