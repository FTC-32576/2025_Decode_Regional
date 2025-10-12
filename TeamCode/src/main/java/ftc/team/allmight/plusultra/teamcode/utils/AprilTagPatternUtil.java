package ftc.team.allmight.plusultra.teamcode.utils;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import ftc.team.Java_Is_AllMight.Sensors.LimelightHelper;

public class AprilTagPatternUtil {

    private final LimelightHelper limelight;
    private Pattern detectedPattern = null; // Assume PPG

    public enum Pattern{
        PPG, PGP, GPP
    }

    public AprilTagPatternUtil(LimelightHelper limelight) {
        this.limelight = limelight;
    }

    public Pattern detectPattern(Telemetry telemetry){

        limelight.start();
        List<LLResultTypes.FiducialResult> fiducials = limelight.getFiducials();

        if (fiducials == null || fiducials.isEmpty()){
            telemetry.addData("AprilTag Pattern", "Not detected"); telemetry.update();
            return null;
        }

        int id = fiducials.get(0).getFiducialId();
        switch (id) {
            case 23: detectedPattern = Pattern.PPG; break;
            case 22: detectedPattern = Pattern.PGP; break;
            case 21: detectedPattern = Pattern.GPP; break;
            default: detectedPattern = Pattern.PPG; break;
        };

        telemetry.addData("AprilTag Pattern", detectedPattern);
        telemetry.update();
        limelight.stop();

        return null;
    }

    public Pattern getDetectedPattern() {
        return detectedPattern;
    }

}
