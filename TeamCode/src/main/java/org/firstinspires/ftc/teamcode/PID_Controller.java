package org.firstinspires.ftc.teamcode;

public class PID_Controller implements PIDInterface{
    final PIDModes mode;
    double integral, previousError, goal1, goal2;
    long lastCheckedI, lastCheckedD;
    public PID_Controller(PIDModes mode){
        this.mode = mode;
    }
    @Override
    public void reset(double x, double y){
        integral = previousError = lastCheckedI = lastCheckedD = 0;
        this.goal1=x;
        this.goal2=y;
    }
    @Override
    public void reset(double heading){
        integral = previousError = lastCheckedI = lastCheckedD = 0;
        this.goal1=heading;
    }
    @Override
    public void setTime(long time, double error){
        this.lastCheckedI=time;
        this.lastCheckedD=time;
        this.previousError = error;
    }
    @Override
    public double getError(double x, double y){
        return errorCalculator(new double[]{x,y});
    }
    @Override
    public double errorCalculator(double[] input) {
        return mode.getError(input, input.length==1?new double[] {this.goal1}:new double[]{this.goal1, this.goal2});
    }
    public double getError(double heading){
        return errorCalculator(new double[]{heading});
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
