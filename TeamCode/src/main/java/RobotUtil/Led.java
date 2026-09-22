package RobotUtil;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

final class Led extends Robot{
    private Led(){super(null, null);}

    @Override
    protected RevBlinkinLedDriver LEDSetUP(OpMode op){
        return op.hardwareMap.get(RevBlinkinLedDriver.class, CONSTANTS.LED);
    }

}
