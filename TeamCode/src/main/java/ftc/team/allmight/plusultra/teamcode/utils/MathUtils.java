package ftc.team.allmight.plusultra.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MathUtils {

    private static double calcularDistanciaAteOGoal(double dY, double dX){
        double d = Math.sqrt(Math.pow(dY, 2) + Math.pow(dX, 2));

        return d;
    }

    private static double arcoSeno(double catetoOposto, double hipotenusa){
        double aSin = Math.asin(catetoOposto/hipotenusa);
        return aSin;
    }

    public static ResultadoMira calcularMira(Pose2D posicaoDoRobo){
        double dX = posicaoDoRobo.getX(DistanceUnit.CM);
        double dY = posicaoDoRobo.getY(DistanceUnit.CM);
        double heading = posicaoDoRobo.getHeading(AngleUnit.DEGREES);

        double d = calcularDistanciaAteOGoal(dY,dX);
        double arcoSeno = arcoSeno(dY, d);

        return new ResultadoMira(arcoSeno, d, heading);

    }

    /**
     * Calcula a potência necessária para o shooter com base na distância.
     * Fórmula: power = a * d + b
     */
    public static double CalculateShooterPower(double distancia) {
        // valores base (ajustáveis conforme testes do robô)
        double a = 0.01;  // fator de crescimento da potência
        double b = 0.4;   // potência mínima
        double potencia = a * distancia + b;

        // limita entre 0 e 1 (faixa válida para motores)
        potencia = Math.min(Math.max(potencia, 0), 1);

        return potencia;
    }


}
