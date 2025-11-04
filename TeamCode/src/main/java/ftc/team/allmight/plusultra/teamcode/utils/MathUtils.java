package ftc.team.allmight.plusultra.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MathUtils {

    // Dimensões do campo em centímetros
    // Usadas para calcular a distância máxima que o robô pode estar do goal
    private static final double FIELD_WIDTH = FieldUtils.FIELD_WIDTH;   // largura do campo
    private static final double FIELD_HEIGHT = FieldUtils.FIELD_HEIGHT; // altura do campo

    // Calcula a diagonal da arena: distância máxima do robô no canto inferior direito
    // até o goal no canto superior esquerdo
    private static final double MAX_DISTANCE = Math.sqrt(FIELD_WIDTH * FIELD_WIDTH + FIELD_HEIGHT * FIELD_HEIGHT);

    // Potência mínima e máxima do shooter (valores ajustáveis)
    private static final double MIN_POWER = 0.4;  // potência mínima quando muito próximo do goal
    private static final double MAX_POWER = 1.0;  // potência máxima quando na distância máxima

    /**
     * Calcula a distância do robô até o goal usando o teorema de Pitágoras
     * @param dY diferença em Y entre o robô e o goal
     * @param dX diferença em X entre o robô e o goal
     * @return distância em cm
     */
    public static double calcularDistanciaAteOGoal(double dY, double dX){
        return Math.sqrt(Math.pow(dY, 2) + Math.pow(dX, 2));
    }

    /**
     * Calcula o arco seno (ângulo em radianos) a partir do cateto oposto e da hipotenusa
     * @param catetoOposto valor do cateto oposto do triângulo
     * @param hipotenusa valor da hipotenusa do triângulo
     * @return ângulo em radianos
     */
    private static double arcoSeno(double catetoOposto, double hipotenusa){
        return Math.asin(catetoOposto / hipotenusa);
    }

    /**
     * Calcula informações da mira do robô com base na sua posição
     * @param posicaoDoRobo posição atual do robô (x, y e heading)
     * @return ResultadoMira contendo o ângulo do arco seno, distância e heading
     */
    public static ResultadoMira calcularMira(Pose2d posicaoDoRobo){
        double dX = posicaoDoRobo.getX();         // posição X do robô
        double dY = posicaoDoRobo.getY();         // posição Y do robô
        double heading = posicaoDoRobo.getHeading(); // orientação do robô em radianos

        // Distância direta do robô até o goal
        double d = calcularDistanciaAteOGoal(dY, dX);

        // Ângulo do triângulo formado pelo robô e o goal
        double angleToGoal = arcoSeno(dY, d);

        return new ResultadoMira(angleToGoal, d, heading);
    }

    /**
     * Calcula a potência necessária do shooter com base na distância até o goal
     * Usa uma relação linear entre MIN_POWER e MAX_POWER
     * @param distanceMeters distância do robô até o goal em metros
     * @return potência entre 0 e 1
     */
    public static double calculateShooterPower(double distanceMeters) {
        // Intervalos calibráveis
        double minDistance = 0.85;   // metros
        double maxDistance = 2.3;   // metros
        double minPower = 0.55;     // potência mínima
        double maxPower = 1.00;     // potência máxima

        // Limita a distância ao range permitido
        double clampedDistance = Math.max(minDistance, Math.min(distanceMeters, maxDistance));

        // Interpolação linear (pode ser substituída por modelo físico depois)
        double normalized = (clampedDistance - minDistance) / (maxDistance - minDistance);
        double power = minPower + normalized * (maxPower - minPower);

        return Math.min(Math.max(power, 0.0), 1.0);
    }
}

