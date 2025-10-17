package ftc.team.allmight.plusultra.teamcode.utils;

public class ResultadoMira {

    public double angulo, distancia, heading;

    public ResultadoMira(double angulo, double distancia, double heading){
        this.angulo = angulo;
        this.distancia = distancia;
        this.heading = heading;
    }

    @Override
    public String toString(){
        return String.format("Angulo: %.2f°, Distancia: %.2f cm, Heading: %.2f°", angulo, distancia, heading);

    }
}
