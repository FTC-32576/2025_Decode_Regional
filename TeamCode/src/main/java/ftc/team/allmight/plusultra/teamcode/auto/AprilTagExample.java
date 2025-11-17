package ftc.team.allmight.plusultra.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import ftc.team.Java_Is_AllMight.Sensors.CameraMight;

@TeleOp(name = "Teste da Webcam")
public class AprilTagExample extends OpMode {

    CameraMight cameraMight = new CameraMight();

    @Override
    public void init() {

        cameraMight.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        cameraMight.update();

        AprilTagDetection id20 = cameraMight.getTagBySpecificId(20);
        cameraMight.DisplayDetectionTelemetry(id20);

//        telemetry.addData("id20 String", id20.toString());

    }
}
