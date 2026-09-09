package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;


@Autonomous
public class Auto_prolly_BLUE extends LinearOpMode {

    @Override
    public void runOpMode(){

        //Start by initializing all the cameras, motors, and also the pinpoint

        //camera 1 and 2
        Limelight3A limelight = Cameras.setupLimeLight(this);
        //HuskyLens huskyLens = Cameras.setupHuskyLens(this);

        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = Pinpoint.setUpPinpoint(this);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,63, AngleUnit.DEGREES,0));
        pinpoint.update();

        //driving motors
        List<DcMotor> motors = Motors.setupDrivingMotors(this);

        //put in all auto modes
        while(opModeInInit()){
            Cameras.placementScanner(limelight, this);
        }


        //won't move on till u click start
        waitForStart();

        //run this code once
        if (opModeIsActive()){
            //drive to top left corner
            PID_Systems.headTo(pinpoint, limelight, motors, this, DistanceUnit.INCH, AngleUnit.DEGREES, 60, 60, 0);
            sleep(1000);

            //drive to top right corner
            PID_Systems.headTo(pinpoint, limelight, motors, this, DistanceUnit.INCH, AngleUnit.DEGREES, 60, -60, 0);
            sleep(1000);

            //drive to bottom right corner
            PID_Systems.headTo(pinpoint, limelight, motors, this, DistanceUnit.INCH, AngleUnit.DEGREES, 0, 0, -45);
            sleep(1000);

            //drive back to start -45
            PID_Systems.headTo(pinpoint, limelight, motors, this, DistanceUnit.INCH, AngleUnit.DEGREES, -60, -60, -45);
            sleep(1000);

            //look for an April tag and lock move until ur exactly 3ft away from it
            PID_Systems.lockIn(DistanceUnit.INCH,this, limelight, pinpoint, motors,36);
        }

    }
}
