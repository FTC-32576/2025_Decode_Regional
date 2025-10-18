package ftc.team.allmight.plusultra.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import ftc.team.Java_Is_AllMight.Utils.Alliance;

public class FieldUtils {

    // Dimensões do campo em cm
    private static final double FIELD_WIDTH = 365.76;
    private static final double FIELD_HEIGHT = 365.76;

    // Poses do goal para cada aliança
    private static final Pose2d RED_GOAL_POSE  = new Pose2d(FIELD_WIDTH - 10, FIELD_HEIGHT - 10, Math.toRadians(-45));
    private static final Pose2d BLUE_GOAL_POSE = new Pose2d(15, FIELD_HEIGHT - 15, Math.toRadians(45));

    public static Pose2d getGoalPose(Alliance alliance) {
        return (alliance == Alliance.RED) ? RED_GOAL_POSE : BLUE_GOAL_POSE;
    }
}
