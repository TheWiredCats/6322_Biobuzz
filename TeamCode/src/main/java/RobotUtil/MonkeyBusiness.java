package RobotUtil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
