package RobotUtil;

public class PID_Controller implements PIDInterface{
    /**Stores the current PID mode using the enum as a global variable*/
    final PIDModes mode;

    /**Stores the PID modes integral as it changes*/
    double integral;

    /**Stores the PID modes previous error as it changes (for derivative)*/
    double previousError;

    /**Stores the last time the PID calculated integral for a more accurate dx (for integral)
     * @see #lastCheckedD Derivatives last checked*/
    long lastCheckedI;

    /** Store the last time the PID calculated derivative for a more accurate dx (for derivative)
     *@see #lastCheckedI Integrals last checked*/
    long lastCheckedD;

    /**Stores the input to give to the enum calculator to not have to overload*/
    double[] input;

    /**Stores the goal position as a global variable and an array to be able to handle the 2D driving position goals as well as the heading goal*/
    double[] goal;


    public PID_Controller(PIDModes mode, RobotUtil calc){
        this.mode = mode;
        this.mode.calc = calc;
        this.input = new double[mode.ordinal()+1];
        this.goal = new double[mode.ordinal()+1];
    }
    /**
     * Sets the goal variables to the inputs and resets the global variables back to 0.
     * <p>Has an overload for turning PIDs
     * @see #reset(double heading) Turning PID reset() Overload
     * @see #goal Goal Variable
     * @param x takes any x between the interval [-72, 72] but should usually be between
     *          [-63,63] to account for robot width/length
     * @param y takes any y between the interval [-72, 72] but should usually be between
     *          [-63,63] to account for robot width/length
     */
    @Override
    public void reset(double x, double y){
        integral = previousError = lastCheckedI = lastCheckedD = 0;
        this.goal[0] = x;
        this.goal[2] = y;
    }

    /**
     * Sets the goal variable to the input, and resets the global variables back to 0.
     * <p>Has an overload for driving PIDs
     * @see #reset(double, double) Driving PID reset() Overload
     * @see #goal Goal Variable
     * @param heading takes any heading between the interval [-179, 180] and sets it as the goal
     */
    @Override
    public void reset(double heading){
        integral = previousError = lastCheckedI = lastCheckedD = 0;
        this.goal[0] = heading;
    }

    /**
     * Sets the time and starting error to be used for calculus calculations
     * @see #lastCheckedI Integral Timer
     * @see #lastCheckedD Derivative Timer
     * @see #previousError Derivative's Previous Error
     * @param time should be the current time and sets lastCheckedI and D to it
     * @param error should be he current error and sets previousError to it
     */
    @Override
    public void setTime(long time, double error){
        this.lastCheckedI = time;
        this.lastCheckedD = time;
        this.previousError = error;
    }

    /**
     * Calculates the current error using the enum specific getError() calculator for driving
     * <p>Has an Overload version for Turning PIDs
     * @see #getError(double heading) Turning PID getError() Overload
     * @param x Should be the current X and uses the preset goal X variable to calculate current error
     * @param y Should be the current Y and uses the preset goal Y variable to calculate current error
     * @return Current Position Error
     * @see PIDModes#getError(double[], double[]) ENUM Error Calculator
     */
    @Override
    public double getError(double x, double y){
        this.input[0]=x;
        this.input[1]=y;
        return mode.getError(this.input, this.goal);
    }

    /**Calculates the current error using the enum specific getError() calculator for turning
     * <p>Has an overload version for Driving PIDs
     * @param heading Should be the current heading and uses the preset goal heading variable to calculate current error
     * @return Current Heading Error
     * @see #getError(double, double) Driving PID getError() Overload
     * @see PIDModes#getError(double[], double[]) ENUM Error Calculator
     */
    @Override
    public double getError(double heading){
        this.input[0]=heading;
        return mode.getError(this.input, goal);
    }

    /**Calculates the P part of the PID by multiplying the enum's PID specific {@linkplain PIDModes#KP P Constant} by the current error
     *@param error The Current Error
     *@see #getError(double) Heading Error Calculator
     *@see #getError(double, double) Position Error Calculator
     *@see #getD(double) D Part of PID
     *@see #getI(double) I Part of PID
     *@return Returns the current P part of the PID
     */
    @Override
    public double getP(double error) {
        return error * this.mode.KP;
    }

    /**
     * Calculates the I part of the PID by integrating the error over time every loop and then multiplying by
     * the {@linkplain PIDModes#KI I Constant}
     * @param error The difference between where I am and where i wanna be
     * @return The I part of the PID System
     * @see #getP(double) P Part of PID System
     * @see #getD(double) D Part of PID System
     */
    @Override
    public double getI(double error) {
        long now = System.nanoTime();
        double dt = Math.max(0.001,((now-this.lastCheckedI)/1000000000.0));
        this.integral+=error*dt;
        this.lastCheckedI = now;
        return this.integral*this.mode.KI;
    }

    /**
     * Calculates the current D part of the PID by getting time plugging into the calculator then multiplying
     * by the {@linkplain  PIDModes#KD D Constant}, and finally saving the current time as lastCheckedD,
     * and the error as previous error
     * @param error The Current Position/Heading Error
     * @see #getError(double) The Position Error Calculator
     * @see #getError(double, double) The Heading Error Calculator
     * @see PIDModes#getD(double, double, double) PID Specific D Calculator
     * @see #getP(double) P Part of PID
     * @see #getI(double) I part of PID
     * @return Returns the current D part of the PID
     */
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
