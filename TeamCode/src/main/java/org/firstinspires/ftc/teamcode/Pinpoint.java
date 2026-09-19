package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//quick set up, and so I can add extra stuff if I ever decide to do smth with it
public final class Pinpoint {
    /**
     * An upgrade of the actual pinpoint's .getPosition
     * @see GoBildaPinpointDriver#getPosition() Actual getPosition()
     * @see #setUpPinpoint(OpMode) Pinpoint Setup
     * @param pinpoint the pinpoint being used
     * @return The Current Position in Units Described By {@linkplain CONSTANTS}
     */
    public static Pose2D getPosition(GoBildaPinpointDriver pinpoint){
        return (new Pose2D(CONSTANTS.unit.DU, pinpoint.getPosX(CONSTANTS.unit.DU),
                pinpoint.getPosY(CONSTANTS.unit.DU), CONSTANTS.unit.AU,
                pinpoint.getHeading(CONSTANTS.unit.AU)));
    }

    /**
     * Adds pinpoint telemetry: Distance Units, Angle Units, X Position, Y Position, Heading
     * @param pinpoint The pinpoint being used
     * @param op Needed to add to telemetry
     */
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

    /**
     * Sets up Pinpoint and gives it, it's default data
     * @param op Needed to access hardwareMap
     * @return The Complete Pinpoint
     */
    public static GoBildaPinpointDriver setUpPinpoint(OpMode op){
        GoBildaPinpointDriver pinpoint = op.hardwareMap.get(GoBildaPinpointDriver.class, CONSTANTS.PINPOINT);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        return pinpoint;
    }
}
