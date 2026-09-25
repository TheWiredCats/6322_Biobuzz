package RobotUtil;

public enum PIDModes {
    TURNING(CONSTANTS.turningConstants) {
        @Override
        public double getError(double[] input, double[] goal) {
            return calc.wrapAngle(goal[0], input[0]);
        }
        @Override
        public double getD(double error, double previousError, double dt){
            return (calc.wrapAngle(error, previousError))*this.KD/dt;
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

    /**
     * P Constant for Either Driving or Turning PID System
     * @see CONSTANTS#driverConstants Driving Constants
     * @see CONSTANTS#turningConstants Turning Constants
     */

    final double KP;
    /**I Constant for Either Driving or Turning PID System
     * @see CONSTANTS#driverConstants Driving Constants
     * @see CONSTANTS#turningConstants Turning Constants
     */
    final double KI;

    /**D Constant for Either Driving or Turning PID System
     * @see CONSTANTS#driverConstants Driving Constants
     * @see CONSTANTS#turningConstants Turning Constants
     */
    final double KD;

    RobotUtil calc;

    PIDModes(CONSTANTS.tunerHolder constants){
        KP=constants.KP;
        KI=constants.KI;
        KD=constants.KD;
    }

    /**Gets error by running one of 2 ways under its implementations
     * @see #DRIVING Driving PID
     * @see #TURNING Turning PID
     * @param input The current position/heading of the robot
     * @param goal The preset position/heading of the goal
     * @return The current difference between the 2, or in other word, the <b>error</b>
     */
    public abstract double getError(double[] input, double[] goal);

    /**
     * Uses limit notation to find the derivative for either the normal error
     * or in turning's case, the wrapped angle error, which is just error with the bounds of {@code [-179,180]}
     * @see RobotUtil#wrapAngle(double, double) Angle Wrapper Calculator
     * @param error Current Error
     * @param previousError The Previously Calculated Error
     * @param dt Difference in time between the current and last calculations
     * @return The rate of change at a single point in the graph, or in other words the <b>derivative</b> of it
     */
    public abstract double getD(double error, double previousError, double dt);

}