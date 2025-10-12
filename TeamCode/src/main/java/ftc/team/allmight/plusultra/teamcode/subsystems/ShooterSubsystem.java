package ftc.team.allmight.plusultra.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private final DcMotor shooterMotor;

    public ShooterSubsystem(HardwareMap hardwareMap, String shooterName){
        shooterMotor = hardwareMap.get(DcMotor.class, shooterName);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(double power){
        shooterMotor.setPower(power);
    }

    public void stop(){
        shooterMotor.setPower(0);
    }

    public  void setDirection(DcMotorSimple.Direction direction){
        shooterMotor.setDirection(direction);
    }
}
