package ftc.team.allmight.plusultra.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import ftc.team.Java_Is_AllMight.Sensors.AprilTagWebcam;

@TeleOp(name = "Teste da Webcam")
public class AprilTagExample extends OpMode {

    AprilTagWebcam aprilTagWebcam  = new AprilTagWebcam();

    @Override
    public void init() {

        aprilTagWebcam.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        aprilTagWebcam.update();

        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.DisplayDetectionTelemetry(id20);

//        telemetry.addData("id20 String", id20.toString());

    }
}
