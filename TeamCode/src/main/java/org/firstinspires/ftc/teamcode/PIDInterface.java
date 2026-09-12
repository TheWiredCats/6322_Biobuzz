package org.firstinspires.ftc.teamcode;


public interface PIDInterface {
    double getError(double a, double b);
    double getError(double Heading);
    double getP(double error);
    double getI(double error);
    double getD(double error);
    void setTime(long time, double error);
    void reset(double x, double y);
    void reset(double heading);

}
