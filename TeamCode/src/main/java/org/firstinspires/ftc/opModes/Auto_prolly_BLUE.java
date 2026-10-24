package org.firstinspires.ftc.opModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import robotUtil.*;

@SuppressWarnings("all")
@Disabled
//@Autonomous
public class Auto_prolly_BLUE extends LinearOpMode {

    @Override
    public void runOpMode(){
        Robot robot = Robot.startAuto(Team.BLUE, this);
        RobotUtil calc = robot.getCalc();

        //Start by initializing all the cameras, motors, and also the pinpoint


        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,63, AngleUnit.DEGREES,0));
        pinpoint.update();

        Limelight3A limelight = robot.getLimelight();


        //put in all auto modes
        //while(opModeInInit()){}


        //won't move on till u click start
        waitForStart();

        //run this code once
        if (opModeIsActive()){

        }

    }
}
