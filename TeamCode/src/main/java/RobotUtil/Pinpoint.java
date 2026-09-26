package RobotUtil;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//quick set up, and so I can add extra stuff if I ever decide to do smth with it
final class Pinpoint extends Robot{
    private Pinpoint(OpMode op){super(op);}
    private static Pinpoint instance;
    static Pinpoint getPinpoint(OpMode op){return instance==null?instance=new Pinpoint(op):instance;}

    /**
     * Sets up Pinpoint and gives it, it's default data
     * @param op Needed to access hardwareMap
     * @return The Complete Pinpoint
     */
    @Override
    protected GoBildaPinpointDriver setUpPinpoint(OpMode op){
        GoBildaPinpointDriver pinpoint = op.hardwareMap.get(GoBildaPinpointDriver.class, CONSTANTS.PINPOINT);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        return pinpoint;
    }
}
