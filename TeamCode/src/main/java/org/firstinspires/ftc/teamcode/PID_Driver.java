package org.firstinspires.ftc.teamcode;

public class PID_Driver extends PID_Controller{
    @Override
    public double getError(double a, double b) {
        return Math.sqrt(Math.pow(a,2)+Math.pow(b,2));
    }
}
