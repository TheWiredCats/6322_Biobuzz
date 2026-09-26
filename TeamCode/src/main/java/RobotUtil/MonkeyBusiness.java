package RobotUtil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * <h1>Personal Error</h1>
 * Comes in 3 flavors,<br>
 * {@linkplain #MonkeyBusiness() Base Error} Which is only for returning in methods,<br>
 * {@linkplain #MonkeyBusiness(String, OpMode) Advanced Error} Which is for stopping the robot <b><i><u>without</u></i></b> having to restart the robot,<br>
 * And {@linkplain #MonkeyBusiness(String) Super Error} Which is for stopping the robot <b><i><u>with</u></i></b> having to restart the robot
 */
public class MonkeyBusiness extends RuntimeException {
    public MonkeyBusiness() {}
    public MonkeyBusiness(String str, OpMode op) {
        op.telemetry.clear();
        op.telemetry.addLine(str);
        op.telemetry.update();
        op.requestOpModeStop();
    }
    public MonkeyBusiness(String str){
        super(str);
    }
}
