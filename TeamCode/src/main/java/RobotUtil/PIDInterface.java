package RobotUtil;


interface PIDInterface {
    double getError(double...input);
    double getP(double error);
    double getI(double error);
    double getD(double error);
    void setTime(long time, double error);
    void reset(double...input);

}
