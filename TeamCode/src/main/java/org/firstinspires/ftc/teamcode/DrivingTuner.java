package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@Autonomous
public class DrivingTuner extends LinearOpMode {
    @Override
    public void runOpMode() {

        //HuskyLens huskyLens = Cameras.setupHuskyLens(this);

        //driving motors
        List<DcMotor> motors = Motors.setupDrivingMotors(this);



        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = Pinpoint.setUpPinpoint(this);
        pinpoint.setPosition(new Pose2D(CONSTANTS.unit.DU, -63, 63, CONSTANTS.unit.AU, 0));
        pinpoint.update();

        Limelight3A limelight = Cameras.setupLimeLight(this, pinpoint);

        RevBlinkinLedDriver LED = Led.LEDSetUP(this);

        //put in all auto modes
        while(opModeInInit()){
            Led.placementScanner(limelight, LED);
        }

        waitForStart();

        //if(opModeIsActive()) Driving_Systems.goTo(pinpoint, limelight, motors, this, CONSTANTS.unit.DU, 21, 12);
    }
}
