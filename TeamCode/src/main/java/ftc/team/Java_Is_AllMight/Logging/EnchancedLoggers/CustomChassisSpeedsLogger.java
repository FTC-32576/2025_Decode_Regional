package ftc.team.Java_Is_AllMight.Logging.EnchancedLoggers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import ftc.team.Java_Is_AllMight.Logging.ChassisSpeed;
import ftc.team.Java_Is_AllMight.Logging.ChassisSpeedsLogEntry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
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


