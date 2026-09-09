package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@Autonomous
public class DrivingTuner extends LinearOpMode {
    @Override
    public void runOpMode() {

        Limelight3A limelight = Cameras.setupLimeLight(this);
        //HuskyLens huskyLens = Cameras.setupHuskyLens(this);

        //driving motors
        List<DcMotor> motors = Motors.setupDrivingMotors(this);



        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = Pinpoint.setUpPinpoint(this);
        pinpoint.setPosition(new Pose2D(CONSTANTS.DISTANCE, -63, 63, CONSTANTS.ANGLE, 0));

        pinpoint.update();

        //put in all auto modes
        while(opModeInInit()){
            Cameras.placementScanner(limelight, this);
        }

        waitForStart();

        if(opModeIsActive()) PID_Systems.goTo(pinpoint, limelight, motors, this, CONSTANTS.DISTANCE, 21, 12);
    }
}
