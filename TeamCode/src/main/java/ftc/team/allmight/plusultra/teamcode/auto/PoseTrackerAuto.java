package ftc.team.allmight.plusultra.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import ftc.team.allmight.plusultra.teamcode.roadrunner.TankDrive;

@Autonomous(name = "Pose2D Tracker (TankDrive RR)", group = "Test")
public class PoseTrackerAuto extends LinearOpMode {

    private TankDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Inicializando TankDrive...");
        telemetry.update();

        // instancie o TankDrive (usa o construtor que você forneceu)
        // o pose inicial setado aqui é (0,0,0) — ajuste se quiser outro.
        drive = new TankDrive(hardwareMap, new Pose2d(new Vector2d(0.0, 0.0), 0.0));

        // Opcional: garanta que o localizer comece na pose 0
        drive.localizer.setPose(new Pose2d(new Vector2d(0.0, 0.0), 0.0));

        drive.updatePoseEstimate(); // uma atualização inicial

        telemetry.clearAll();
        telemetry.addLine("✅ Pose2D Tracker pronto");
        telemetry.addLine("Empurre o robô com a mão para ver a pose atualizar");
        telemetry.addLine("Pressione PLAY para iniciar");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Loop principal: atualiza a pose e imprime telemetria
        final long loopMs = 50L;
        while (opModeIsActive()) {
            // atualiza estimativa de pose (localizer.update() dentro)
            PoseVelocity2d vel = drive.updatePoseEstimate();

            // pega a pose atual
            Pose2d pose = drive.localizer.getPose();

            // pega ticks crus para diagnóstico (assumindo pelo menos um motor por lado)
            int leftTicks = 0, rightTicks = 0;
            try {
                leftTicks = drive.leftMotors.get(0).getCurrentPosition();
                rightTicks = drive.rightMotors.get(0).getCurrentPosition();
            } catch (Exception ignored) {
                // se falhar, deixamos 0 e continuamos (não quebra o opmode)
            }

            // preparar telemetria bonita
            telemetry.clearAll();
            telemetry.addLine("📍 POSE ATUAL");
            telemetry.addData("X (m)", "%.3f", pose.position.x);
            telemetry.addData("Y (m)", "%.3f", pose.position.y);

            double headingRad = pose.heading.toDouble();
            telemetry.addData("Heading (rad)", "%.3f", headingRad);
            telemetry.addData("Heading (°)", "%.1f", Math.toDegrees(headingRad));

            telemetry.addLine();
            telemetry.addLine("⚙️ ENCODERS (crus)");
            telemetry.addData("Left ticks", leftTicks);
            telemetry.addData("Right ticks", rightTicks);

            telemetry.addLine();
            telemetry.addLine("📈 VELOCIDADES ESTIMADAS");
            if (vel != null) {
                telemetry.addData("Vel linear (m/s)", "%.3f", vel.linearVel);
                telemetry.addData("Vel angular (rad/s)", "%.3f", vel.angVel);
            } else {
                telemetry.addData("Vel", "null");
            }

            // checar NaN e avisar
            boolean invalid = !Double.isFinite(pose.position.x) ||
                    !Double.isFinite(pose.position.y) ||
                    !Double.isFinite(headingRad);

            if (invalid) {
                telemetry.addLine();
                telemetry.addLine("⚠️ POSE INVÁLIDA (NaN) DETECTADA!");
                telemetry.addLine("Verifique: nomes dos motores, IMU, e parâmetros de inPerTick/trackWidth");
            }

            telemetry.addLine();
            telemetry.addLine("Dicas:");
            telemetry.addLine(" - Confirme nomes: motorEsquerdo / motorDireito");
            telemetry.addLine(" - Verifique IMU mounting / orientation se heading estiver bizarro");
            telemetry.update();

            sleep(loopMs);
        }
    }
}
