package ftc.team.Java_Is_AllMight.Config;

/**
 * Filtro simples para fusão de pose 2D (x, y, heading) baseado em um
 * Kalman linear simplificado (estimativa e correção por observação de pose).
 *
 * Notas:
 * - Estado: [x, y, heading]
 * - Modelo de processo tratado como identidade + ruído de processo (Q).
 * - Observação: pose completa (z = [x, y, heading]) com covariância R.
 *
 * Este filtro NÃO é um EKF completo — é suficiente para fundir leituras de odo/IMU
 * com correções de pose (por ex. Limelight) em muitos cenários FTC.
 */
public class KalmanFilter {

    // Estado
    private double x;
    private double y;
    private double heading; // radianos

    // Covariância do estado (3x3) — armazenada plano:
    private double P00, P01, P02;
    private double P10, P11, P12;
    private double P20, P21, P22;

    // Ruído do processo (Q) — confiança na modelagem da odometria (valores pequenos = confiamos)
    private double qPos;   // para x,y
    private double qHeading; // para heading

    public KalmanFilter(double initX, double initY, double initHeading,
                                  double initCovPos, double initCovHeading,
                                  double qPos, double qHeading) {
        this.x = initX;
        this.y = initY;
        this.heading = initHeading;
        // inicializa P diagonal
        this.P00 = initCovPos; this.P01 = 0; this.P02 = 0;
        this.P10 = 0; this.P11 = initCovPos; this.P12 = 0;
        this.P20 = 0; this.P21 = 0; this.P22 = initCovHeading;

        this.qPos = qPos;
        this.qHeading = qHeading;
    }

    /**
     * Predição pelo modelo de movimento incremental: aplica um deslocamento delta (do odômetro)
     * deltaX, deltaY (em campo) e deltaHeading (rad). A covariância é expandida por Q.
     */
    public void predict(double deltaX, double deltaY, double deltaHeading) {
        // atualizar estado
        this.x += deltaX;
        this.y += deltaY;
        this.heading = normalizeAngle(this.heading + deltaHeading);

        // Atualiza P = P + Q (Q diagonal)
        P00 += qPos;
        P11 += qPos;
        P22 += qHeading;
    }

    /**
     * Correção usando observação completa de pose (zX, zY, zHeading)
     * rPos = variância da posição observada (m^2)
     * rHeading = variância da heading observada (rad^2)
     */
    public void correct(double zX, double zY, double zHeading, double rPos, double rHeading) {
        // Matriz R (observação cov)
        double R00 = rPos, R11 = rPos, R22 = rHeading;

        // Inovação y = z - Hx  (H = I)
        double y0 = zX - x;
        double y1 = zY - y;
        double y2 = shortestAngularDifference(zHeading, heading);

        // S = P + R  (porque H = I)
        double S00 = P00 + R00;
        double S01 = P01; // off diag P01 + 0
        double S02 = P02;
        double S10 = P10;
        double S11 = P11 + R11;
        double S12 = P12;
        double S20 = P20;
        double S21 = P21;
        double S22 = P22 + R22;

        // Compute inverse of S (3x3) — closed form (adjugate/determinant)
        double detS = computeDeterminant3x3(
                S00, S01, S02,
                S10, S11, S12,
                S20, S21, S22
        );

        if (Math.abs(detS) < 1e-12) {
            // não numericamente estável, ignora correção
            return;
        }

        double invS00 =  (S11*S22 - S12*S21)/detS;
        double invS01 = -(S01*S22 - S02*S21)/detS;
        double invS02 =  (S01*S12 - S02*S11)/detS;
        double invS10 = -(S10*S22 - S12*S20)/detS;
        double invS11 =  (S00*S22 - S02*S20)/detS;
        double invS12 = -(S00*S12 - S02*S10)/detS;
        double invS20 =  (S10*S21 - S11*S20)/detS;
        double invS21 = -(S00*S21 - S01*S20)/detS;
        double invS22 =  (S00*S11 - S01*S10)/detS;

        // Kalman gain K = P * invS (3x3 * 3x3)
        double K00 = P00*invS00 + P01*invS10 + P02*invS20;
        double K01 = P00*invS01 + P01*invS11 + P02*invS21;
        double K02 = P00*invS02 + P01*invS12 + P02*invS22;

        double K10 = P10*invS00 + P11*invS10 + P12*invS20;
        double K11 = P10*invS01 + P11*invS11 + P12*invS21;
        double K12 = P10*invS02 + P11*invS12 + P12*invS22;

        double K20 = P20*invS00 + P21*invS10 + P22*invS20;
        double K21 = P20*invS01 + P21*invS11 + P22*invS21;
        double K22 = P20*invS02 + P21*invS12 + P22*invS22;

        // Atualiza estado: x = x + K * y
        x += K00*y0 + K01*y1 + K02*y2;
        y += K10*y0 + K11*y1 + K12*y2;
        heading = normalizeAngle( heading + (K20*y0 + K21*y1 + K22*y2) );

        // Atualiza covariância: P = (I - K) P
        // Compute (I - K)
        double I_K00 = 1 - K00, I_K01 = -K01,      I_K02 = -K02;
        double I_K10 = -K10,     I_K11 = 1 - K11,  I_K12 = -K12;
        double I_K20 = -K20,     I_K21 = -K21,     I_K22 = 1 - K22;

        double nP00 = I_K00*P00 + I_K01*P10 + I_K02*P20;
        double nP01 = I_K00*P01 + I_K01*P11 + I_K02*P21;
        double nP02 = I_K00*P02 + I_K01*P12 + I_K02*P22;

        double nP10 = I_K10*P00 + I_K11*P10 + I_K12*P20;
        double nP11 = I_K10*P01 + I_K11*P11 + I_K12*P21;
        double nP12 = I_K10*P02 + I_K11*P12 + I_K12*P22;

        double nP20 = I_K20*P00 + I_K21*P10 + I_K22*P20;
        double nP21 = I_K20*P01 + I_K21*P11 + I_K22*P21;
        double nP22 = I_K20*P02 + I_K21*P12 + I_K22*P22;

        P00 = nP00; P01 = nP01; P02 = nP02;
        P10 = nP10; P11 = nP11; P12 = nP12;
        P20 = nP20; P21 = nP21; P22 = nP22;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public void setState(double nx, double ny, double nHeading) {
        x = nx; y = ny; heading = normalizeAngle(nHeading);
    }

    // Helpers
    private static double shortestAngularDifference(double a, double b) {
        // retorna a - b ajustado para -pi..pi
        double d = a - b;
        while (d > Math.PI) d -= 2*Math.PI;
        while (d < -Math.PI) d += 2*Math.PI;
        return d;
    }

    private static double normalizeAngle(double ang) {
        while (ang > Math.PI) ang -= 2*Math.PI;
        while (ang < -Math.PI) ang += 2*Math.PI;
        return ang;
    }

    private static double computeDeterminant3x3(
            double a00,double a01,double a02,
            double a10,double a11,double a12,
            double a20,double a21,double a22) {
        return a00*(a11*a22 - a12*a21)
                - a01*(a10*a22 - a12*a20)
                + a02*(a10*a21 - a11*a20);
    }
}
