package ftc.team.allmight.plusultra.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import ftc.team.Java_Is_AllMight.Utils.RoboUtils;

public class IntakeSubsytem {

    // Estados possíveis do intake durante a partida
    // INTAKE: puxar artefato para dentro
    // STOP: parar motor e não consumir energia
    // REVERSE: cuspir/expelir artefato para liberar entupimento
    public enum IntakeState{
        INTAKE,
        STOP,
        REVERSE
    }

    private final DcMotorEx intakeMotor;
    private IntakeState currentState = IntakeState.STOP;
    private final Telemetry telemetry;

    private final RoboUtils roboUtils = new RoboUtils();

    public IntakeSubsytem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        intakeMotor = roboUtils.getHardware(hardwareMap, DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    // Intenção: puxar artefato
    public void intake(){
        currentState = IntakeState.INTAKE;
    }

    // Intenção: expelir artwefato
    public void reverse(){
        currentState = IntakeState.REVERSE;
    }

    // Intenção: parar motor
    public void stop(){
        currentState = IntakeState.STOP;
    }

    // Aplica o estado no motor (chamado no update)

    private void applyState(){
        switch (currentState){
            case INTAKE: intakeMotor.setPower(-1); break;
            case REVERSE: intakeMotor.setPower(1); break;
            case STOP: default: intakeMotor.setPower(0.0); break;
        }
    }

    public void update(){
        applyState();
        // Logs para debug em campo e no pit
        telemetry.addData("Intake State", currentState);
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Intake Current (A)", intakeMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.update();

    }

    public boolean intakeIsBusy(){
        return intakeMotor.isBusy();
    }

}
