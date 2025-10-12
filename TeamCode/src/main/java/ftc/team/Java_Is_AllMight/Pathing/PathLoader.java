package ftc.team.Java_Is_AllMight.Pathing;

import android.content.Context;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import ftc.team.allmight.plusultra.teamcode.roadrunner.TankDrive;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.List;

public class PathLoader {

    /**
     * Carrega uma trajetória a partir de um arquivo JSON dentro de assets/paths/
     * @param context Contexto do app (hardwareMap.appContext)
     * @param fileName nome do arquivo JSON sem extensão
     * @param startPose Pose inicial do robô
     * @param drive Drive (TankDrive)
     * @return Lista de Trajetórias (Trajectory)
     */
    public static List<Trajectory> loadTrajectory(Context context, String fileName, Pose2d startPose, TankDrive drive) {
        try {
            // Abre o arquivo dentro de assets/paths/
            InputStream is = context.getAssets().open("paths/" + fileName + ".json");
            BufferedReader br = new BufferedReader(new InputStreamReader(is));
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                sb.append(line);
            }
            br.close();

            JSONArray waypoints = new JSONArray(sb.toString());

            // Começamos do startPose
            Pose2d currentPose = startPose;
            TrajectoryActionBuilder builder = drive.actionBuilder(currentPose);

            // Itera pelos waypoints
            for (int i = 0; i < waypoints.length(); i++) {
                JSONObject wp = waypoints.getJSONObject(i);
                double x = wp.getDouble("x");
                double y = wp.getDouble("y");
                double heading = Math.toRadians(wp.getDouble("heading"));

                builder.lineToXLinearHeading(x, heading);
                builder.lineToYLinearHeading(y, heading);
            }

            // Retorna a trajetória pronta
            return (List<Trajectory>) builder.build();

        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
}
