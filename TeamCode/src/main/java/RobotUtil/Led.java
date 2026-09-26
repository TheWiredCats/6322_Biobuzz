package RobotUtil;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

final class Led extends Robot{
    private Led(OpMode op){super(op);}
    private static Led instance;
    static Led getLed(OpMode op){return instance==null?instance=new Led(op):instance;}

    @Override
    protected RevBlinkinLedDriver LEDSetUP(OpMode op){
        return op.hardwareMap.get(RevBlinkinLedDriver.class, CONSTANTS.LED);
    }

}
