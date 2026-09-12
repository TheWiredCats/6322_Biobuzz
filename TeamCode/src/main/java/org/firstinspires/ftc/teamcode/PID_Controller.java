package org.firstinspires.ftc.teamcode;

public class PID_Controller implements PIDInterface{
    final PIDModes mode;
    double integral, previousError;
    long lastCheckedI, lastCheckedD;
    double[] input;
    double[] goal;
    public PID_Controller(PIDModes mode){
        this.mode = mode;
        this.input = new double[mode.ordinal()+1];
        this.goal = new double[mode.ordinal()+1];
    }

    @Override
    public void reset(double x, double y){
        integral = previousError = lastCheckedI = lastCheckedD = 0;
        this.goal[0] = x;
        this.goal[2] = y;
    }

    @Override
    public void reset(double heading){
        integral = previousError = lastCheckedI = lastCheckedD = 0;
        this.goal[0] = heading;
    }

    @Override
    public void setTime(long time, double error){
        this.lastCheckedI = time;
        this.lastCheckedD = time;
        this.previousError = error;
    }

    @Override
    public double getError(double x, double y){
        this.input[0]=x;
        this.input[1]=y;
        return mode.getError(this.input, this.goal);
    }

    @Override
    public double getError(double heading){
        this.input[0]=heading;
        return mode.getError(this.input, goal);
    }

    @Override
    public double getP(double error) {
        return error * this.mode.KP;
    }

    @Override
    public double getI(double error) {
        long now = System.nanoTime();
        double dt = Math.max(0.001,((now-this.lastCheckedI)/1000000000.0));
        this.integral+=error*dt;
        this.lastCheckedI = now;
        return this.integral*this.mode.KI;
    }

    @Override
    public double getD(double error) {
        long now = System.nanoTime();
        double dt = Math.max(0.001,((now-this.lastCheckedD)/1000000000.0));
        double result = mode.getD(error, this.previousError, dt);
        this.lastCheckedD=now;
        this.previousError = error;
        return result;
    }
}
