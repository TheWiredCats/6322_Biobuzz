package org.firstinspires.ftc.teamcode;

public abstract class PID_Controller implements PIDInterface{
    @Override
    public abstract double getError(double a, double b);

    @Override
    public double getP(double error, double KI) {
        return error * KI;
    }
    @Override
    public double[] getI(double error, double dt, double lastI, double KI) {
        dt = Math.max(dt/1000000000,0.001);
        return new double[]{KI*(lastI+(error*dt)),lastI+(error*dt)};
    }
    @Override
    public double getD(double error, double previousError, double dt, double KD) {
        dt = Math.max(dt/1000000000,0.001);
        return (error-previousError)*(KD/dt);
    }
}
