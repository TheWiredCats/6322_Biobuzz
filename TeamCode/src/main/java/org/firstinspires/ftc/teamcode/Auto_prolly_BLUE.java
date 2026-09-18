package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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


        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = Pinpoint.setUpPinpoint(this);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,63, AngleUnit.DEGREES,0));
        pinpoint.update();

        Limelight3A limelight = Cameras.setupLimeLight(this, pinpoint);

        //driving motors
        List<DcMotor> motors = Motors.setupDrivingMotors(this);

        //led setup
        RevBlinkinLedDriver LED = Led.LEDSetUP(this);

        //put in all auto modes
        while(opModeInInit()){
            Led.placementScanner(limelight, LED);
        }


        //won't move on till u click start
        waitForStart();

        //run this code once

        if (opModeIsActive()){
        }

    }
}
