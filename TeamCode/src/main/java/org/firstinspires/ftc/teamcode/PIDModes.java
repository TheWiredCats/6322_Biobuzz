package org.firstinspires.ftc.teamcode;

public enum PIDModes {
    TURNING(CONSTANTS.turningConstants) {
        @Override
        public double getError(double[] input, double[] goal) {
            return Cameras.wrapAngle(CONSTANTS.unit.AU, goal[0]-input[0]);
        }
        @Override
        public double getD(double error, double previousError, double dt){
            return (Cameras.wrapAngle(CONSTANTS.unit.AU, error-previousError))*this.KD/dt;
        }
    },
    DRIVING(CONSTANTS.driverConstants){
        @Override
        public double getError(double[] input, double[] goal) {
            return Math.hypot(goal[0]-input[0], goal[1]-input[1]);
        }
        @Override
        public double getD(double error, double previousError, double dt){
            return (error-previousError)*(this.KD/dt);
        }
    };
    final double KP;
    final double KI;
    final double KD;
    PIDModes(CONSTANTS.tunerHolder constants){
        KP=constants.KP;
        KI=constants.KI;
        KD=constants.KD;
    }
    public abstract double getError(double[] input, double[] goal);
    public abstract double getD(double error, double previousError, double dt);

}