package org.firstinspires.ftc.teamcode;

public abstract class PID_Turner extends PID_Controller {
    @Override
    public double getError(double a, double b){
        return a-b;
    }
}
