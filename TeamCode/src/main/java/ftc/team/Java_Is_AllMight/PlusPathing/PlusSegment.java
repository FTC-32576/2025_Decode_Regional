package ftc.team.Java_Is_AllMight.PlusPathing;

public class PlusSegment {
    public enum Type { MOVE, TURN }

    public final Type type;
    public final double valueCm; // distância em cm (para MOVE)
    public final double angleRad; // ângulo em rad (para TURN)

    public PlusSegment(Type type, double valueCm, double angleRad) {
        this.type = type;
        this.valueCm = valueCm;
        this.angleRad = angleRad;
    }

    public static PlusSegment move(double cm) {
        return new PlusSegment(Type.MOVE, cm, 0);
    }

    public static PlusSegment turn(double degrees) {
        return new PlusSegment(Type.TURN, 0, Math.toRadians(degrees));
    }
}
