package org.firstinspires.ftc.teamcode;


public interface PIDInterface {
    double getError(double a, double b);
    double getP(double error, final double KP);
    double[] getI(double error, double dt ,double lastI, final double KI);
    double getD(double error, double previousError, double dt, double KD);

}
