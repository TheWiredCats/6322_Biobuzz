package RobotUtil;

import java.util.concurrent.TimeUnit;

class PID_Controller implements PIDInterface{
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
    private static final float NANO_PER_SECOND = 1E9f;

    public PID_Controller(PIDModes mode, RobotUtil calc){
        this.mode = mode;
        this.mode.calc = calc;
        this.input = new double[mode.ordinal()+1];
        this.goal = new double[mode.ordinal()+1];
    }
    /**
     * Sets the goal variables to the inputs and resets the global variables back to 0.
     * @see #goal Goal Variable
     * @param input Varg for the goal you want
     */
    @Override
    public void reset(double... input){
        integral = previousError = lastCheckedI = lastCheckedD = 0;
        this.goal=input.clone();
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
     * @param input VArg to make it easier to use, just input the things you want to input
     * @return Current Position Error
     * @see PIDModes#getError(double[], double[]) ENUM Error Calculator
     */
    @Override
    public double getError(double...input){
        this.input=input.clone();
        return mode.getError(this.input, this.goal);
    }

    /**Calculates the P part of the PID by multiplying the enum's PID specific {@linkplain PIDModes#KP P Constant} by the current error
     *@param error The Current Error
     *@see #getError(double...) Error Calculator
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
        this.integral+=error*dtCalculator(now,this.lastCheckedI);
        this.lastCheckedI = now;
        return this.integral*this.mode.KI;
    }

    /**
     * Calculates the current D part of the PID by getting time plugging into the calculator then multiplying
     * by the {@linkplain  PIDModes#KD D Constant}, and finally saving the current time as lastCheckedD,
     * and the error as previous error
     * @param error The Current Position/Heading Error
     * @see PIDModes#getD(double, double, double) PID Specific D Calculator
     * @see #getP(double) P Part of PID
     * @see #getI(double) I part of PID
     * @return Returns the current D part of the PID
     */
    @Override
    public double getD(double error) {
        long now = System.nanoTime();
        double result = mode.getD(error, this.previousError, dtCalculator(now,this.lastCheckedD));
        this.lastCheckedD=now;
        this.previousError = error;
        return result;
    }

    /**
     * Calculates the current dt by subtracting the last
     * @param now Current Time
     * @param last Last Time it was checked
     * @return The time difference
     */
    private float dtCalculator(long now, long last){
        if(last>now)throw new IllegalArgumentException("Last Cant Come After Now");
        if(last==0)throw new IllegalStateException("Make sure that you properly set last before calling this");
        return Math.max(1E-6f,(now-last)/NANO_PER_SECOND);
    }
}
