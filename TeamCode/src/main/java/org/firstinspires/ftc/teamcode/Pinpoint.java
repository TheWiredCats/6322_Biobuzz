package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//quick set up, and so I can add extra stuff if I ever decide to do smth with it
public final class Pinpoint {
    public static void addTelemetry(GoBildaPinpointDriver pinpoint, OpMode op){
        //adds the distance unit, angle unit, x y positions, and the heading to telemetry
        op.telemetry.addData("Odometry Data", "Distance Unit: %s, Angle Unit: %s, " +
                        "X Pos: %.2f, Y Pos: %.2f, Heading: %.2f",
                CONSTANTS.unit.DU,
                CONSTANTS.unit.AU,
                pinpoint.getPosX(CONSTANTS.unit.DU),
                pinpoint.getPosY(CONSTANTS.unit.DU),
                pinpoint.getHeading(CONSTANTS.unit.AU)
        );
    }
    //can just call a single function rather than righting it all out
    public static GoBildaPinpointDriver setUpPinpoint(OpMode op){
        GoBildaPinpointDriver pinpoint = op.hardwareMap.get(GoBildaPinpointDriver.class, CONSTANTS.PINPOINT);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        return pinpoint;
    }
}
