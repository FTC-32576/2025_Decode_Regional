package ftc.team.allmight.plusultra.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

public class AprilTagPatternUtil {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Pattern detectedPattern = null;

    public enum Pattern {
        PPG, PGP, GPP
    }

    public AprilTagPatternUtil(VisionPortal visionPortal, AprilTagProcessor aprilTag) {
        this.visionPortal = visionPortal;
        this.aprilTag = aprilTag;
    }

    public Pattern detectPattern(Telemetry telemetry) {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections == null || detections.isEmpty()) {
            telemetry.addData("AprilTag Pattern", "Not detected");
            telemetry.update();
            return null;
        }

        int id = detections.get(0).id;
        switch (id) {
            case 23: detectedPattern = Pattern.PPG; break;
            case 22: detectedPattern = Pattern.PGP; break;
            case 21: detectedPattern = Pattern.GPP; break;
            default: detectedPattern = Pattern.PPG; break;
        }

        telemetry.addData("AprilTag Pattern", detectedPattern);
        telemetry.update();

        return detectedPattern;
    }

    public Pattern getDetectedPattern() {
        return detectedPattern;
    }
}
