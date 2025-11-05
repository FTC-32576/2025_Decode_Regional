package ftc.team.allmight.plusultra.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoSubsystem {

    private final Servo servo;

    public ServoSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "servidor");
        servo.setPosition(0);
    }

    public void setPosition(double pos) {
        pos = Math.max(0, Math.min(pos, 1));
        servo.setPosition(pos);
    }

    public double getPosition() { return servo.getPosition(); }
}
