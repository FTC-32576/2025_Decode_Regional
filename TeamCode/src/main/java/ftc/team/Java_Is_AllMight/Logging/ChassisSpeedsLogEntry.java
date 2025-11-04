package ftc.team.Java_Is_AllMight.Logging;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class ChassisSpeedsLogEntry{

    protected final String name;
    protected final Telemetry telemetry;

    public ChassisSpeedsLogEntry(String name, Telemetry telemetry) {
        this.name = name;
        this.telemetry = telemetry;
    }

    public void append(ChassisSpeed chassisSpeeds) {
        // Envia direto para telemetry
        telemetry.addData(name + " vx", chassisSpeeds.vx);
        telemetry.addData(name + " vy", chassisSpeeds.vy);
        telemetry.addData(name + " omega", chassisSpeeds.omega);
        telemetry.update();
    }
}
