package ftc.team.Java_Is_AllMight.Logging;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class ChassisSpeed {
    public double vx, vy, omega;

    public ChassisSpeed(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public ChassisSpeed() {
        this(0,0,0);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof ChassisSpeed)) return false;
        ChassisSpeed other = (ChassisSpeed) obj;
        return this.vx == other.vx && this.vy == other.vy && this.omega == other.omega;
    }
}
