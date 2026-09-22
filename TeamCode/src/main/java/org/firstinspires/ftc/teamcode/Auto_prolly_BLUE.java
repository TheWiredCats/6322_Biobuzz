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

import RobotUtil.Robot;
import RobotUtil.RobotUtil;


@Autonomous
public class Auto_prolly_BLUE extends LinearOpMode {

    @Override
    public void runOpMode(){
        Robot robot = Robot.getInstance(this, Team.BLUE);

        //Start by initializing all the cameras, motors, and also the pinpoint


        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,63, AngleUnit.DEGREES,0));
        pinpoint.update();

        Limelight3A limelight = robot.getLimelight();

        //driving motors
        List<DcMotor> motors = robot.getDrivingMotors();

        //led setup
        RevBlinkinLedDriver LED = robot.getLed();

        //put in all auto modes
        while(opModeInInit()){
            RobotUtil.placementScanner(limelight, LED);
        }


        //won't move on till u click start
        waitForStart();

        //run this code once

        if (opModeIsActive()){

        }

    }
}
