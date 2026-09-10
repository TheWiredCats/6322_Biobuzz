package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public final class Led {
    private Led(){}
    public static RevBlinkinLedDriver LEDSetUP(OpMode op){
        return op.hardwareMap.get(RevBlinkinLedDriver.class, CONSTANTS.LED);
    }
    public static void placementScanner(Limelight3A limelight, RevBlinkinLedDriver LED){
        try{
            LLResultTypes.FiducialResult x = Cameras.getBiggest(limelight.getLatestResult());
            if(Math.abs(x.getTargetXDegrees()) > 10) LED.setPattern(CONSTANTS.ledConfig.TARGET_SIGHTED);
            else if(Math.abs(x.getTargetXDegrees()) <= 10 && Math.abs(x.getTargetXDegrees()) > 5)
                LED.setPattern(CONSTANTS.ledConfig.CLOSE);
            else LED.setPattern(CONSTANTS.ledConfig.ON_POINT);
        } catch (NullPointerException ignored) {
            LED.setPattern(CONSTANTS.ledConfig.NONE_SIGHTED);
        }
    }
}
