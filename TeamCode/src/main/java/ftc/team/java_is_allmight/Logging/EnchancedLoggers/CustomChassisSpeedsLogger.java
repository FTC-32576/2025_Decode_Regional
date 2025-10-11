package ftc.team.java_is_allmight.Logging.EnchancedLoggers;

import ftc.team.java_is_allmight.Logging.ChassisSpeed;
import ftc.team.java_is_allmight.Logging.ChassisSpeedsLogEntry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CustomChassisSpeedsLogger extends ChassisSpeedsLogEntry {

    private ChassisSpeed loggedValue;

    public CustomChassisSpeedsLogger(String name, Telemetry telemetry) {
        super(name, telemetry);
        this.loggedValue = new ChassisSpeed(0,0,0); // inicial
        this.append(this.loggedValue); // log inicial
    }

    @Override
    public void append(ChassisSpeed chassisSpeeds) {
        // Só loga se mudou
        if (!chassisSpeeds.equals(this.loggedValue)) {
            this.loggedValue = chassisSpeeds;
            super.append(chassisSpeeds);
        }
    }
}


