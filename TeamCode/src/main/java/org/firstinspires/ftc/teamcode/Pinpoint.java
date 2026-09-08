package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public final class Pinpoint {
    public static GoBildaPinpointDriver setUpPinpoint(OpMode op){
        GoBildaPinpointDriver pinpoint = op.hardwareMap.get(GoBildaPinpointDriver.class, CONSTANTS.PINPOINT);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        return pinpoint;
    }
}
