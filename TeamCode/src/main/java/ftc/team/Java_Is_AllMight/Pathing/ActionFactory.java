package ftc.team.Java_Is_AllMight.Pathing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import ftc.team.allmight.plusultra.teamcode.roadrunner.TankDrive;

import java.util.ArrayList;
import java.util.List;

/**
 * ActionFactory - converte uma lista de Pose2d em Actions usando o drive (TankDrive ou MecanumDrive).
 *
 * Nota: essa fábrica usa métodos simples (lineTo) para construir trajetórias entre pontos.
 * Você pode estender para usar splineTo/lineToLinearHeading/markers conforme precisar.
 */
public final class ActionFactory {

    private ActionFactory() {}

    /**
     * Cria uma lista de Actions (trajetórias) que movem o robô sequencialmente pelos poses.
     *
     * @param drive instância do TankDrive (ou Mecanum equivalente que possui actionBuilder)
     * @param poses lista de poses no sistema do campo (metros / radians)
     * @return lista de Actions prontas para executar com Actions.runBlocking(...)
     */
    public static List<Action> buildActionsFromPoses(TankDrive drive, List<Pose2d> poses) {
        List<Action> actions = new ArrayList<>();
        if (poses == null || poses.size() < 2) return actions;

        // beginPose é o primeiro elemento
        Pose2d beginPose = poses.get(0);

        // para cada target subsequente, crio uma ação (trajetória) do beginPose -> target
        for (int i = 1; i < poses.size(); i++) {
            Pose2d target = poses.get(i);

            // usa o actionBuilder do drive para criar a ação.
            // Aqui usamos lineTo em coordenadas cartesianas (x,y). Se quiser heading, troque por lineToLinearHeading/splineTo.
            Action a = drive.actionBuilder(beginPose)
                    .lineToX(target.position.x)
                    .lineToY(target.position.y)
                    .build();

            actions.add(a);

            // próximo begin é a pose alvo (assume execução completa)
            beginPose = target;
        }

        return actions;
    }
}