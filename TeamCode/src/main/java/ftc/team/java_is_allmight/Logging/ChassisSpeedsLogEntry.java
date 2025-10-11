package ftc.team.java_is_allmight.Logging;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
