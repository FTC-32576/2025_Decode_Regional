package ftc.team.Java_Is_AllMight.Sensors;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.team.Java_Is_AllMight.Utils.DriveConstants;


public class EncoderMight {

    private DcMotorEx motor;
    private Telemetry telemetry;

    // Estado básico
    private int ticks = 0;
    private int lastTicks = 0;
    private long lastTime = 0;

    private double rawVelocity = 0;       // ticks/s
    private double filteredVelocity = 0;  // ticks/s
    private double alpha = 0.25;          // filtro EMA

    // Stall detection
    private boolean stallDetected = false;
    private final double stallThreshold = 5; // ticks/s

    public EncoderMight(HardwareMap hw, Telemetry tele, String motorName) {
        this.motor = hw.get(DcMotorEx.class, motorName);
        this.telemetry = tele;

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lastTime = System.nanoTime();
    }

    public void update() {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        lastTicks = ticks;
        ticks = motor.getCurrentPosition();

        // Velocidade bruta
        rawVelocity = motor.getVelocity();

        // Filtro EMA
        if (filteredVelocity == 0) filteredVelocity = rawVelocity;
        filteredVelocity = alpha * rawVelocity + (1 - alpha) * filteredVelocity;

        // Detecção de stall
        stallDetected = Math.abs(rawVelocity) < stallThreshold;
    }

    // ============================
    // GETTERS UNIVERSAIS
    // ============================

    public int getTicks() {
        return ticks;
    }

    public double getVelocityTicks() {
        return rawVelocity;
    }

    public double getFilteredVelocityTicks() {
        return filteredVelocity;
    }

    public double getAccelerationTicks() {
        return rawVelocity - filteredVelocity;
    }

    public boolean isStalled() {
        return stallDetected;
    }

    // ============================
    // CONVERSÕES PADRÃO (DriveConstants)
    // ============================

    /** Distância em cm */
    public double getDistanceCm() {
        return ticks * DriveConstants.CM_PER_TICK;
    }

    /** Velocidade (cm/s) */
    public double getVelocityCmPerSec() {
        return rawVelocity * DriveConstants.CM_PER_TICK;
    }

    /** Velocidade filtrada (cm/s) */
    public double getFilteredVelocityCmPerSec() {
        return filteredVelocity * DriveConstants.CM_PER_TICK;
    }

    /** Conversão para RPM */
    public double getRPM() {
        return (rawVelocity / DriveConstants.TICKS_PER_REV) * 60.0;
    }

    public double getFilteredRPM() {
        return (filteredVelocity / DriveConstants.TICKS_PER_REV) * 60.0;
    }

    // ============================
    // CONTROLE DO MOTOR
    // ============================

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void brake() {
        motor.setPower(0);
    }

    // ============================
    // RESET
    // ============================

    public void reset() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        ticks = 0;
        rawVelocity = 0;
        filteredVelocity = 0;
    }

    public void log(String tag) {
        telemetry.addLine("=== " + tag + " EncoderMight ===");
        telemetry.addData("Ticks", ticks);
        telemetry.addData("Distance (cm)", this::getDistanceCm);
        telemetry.addData("Vel (t/s)", rawVelocity);
        telemetry.addData("Vel Filtrada (t/s)", filteredVelocity);
        telemetry.addData("RPM", getRPM());
        telemetry.addData("RPM Filtrado", getFilteredRPM());
        telemetry.addData("Stall", stallDetected);
    }
}