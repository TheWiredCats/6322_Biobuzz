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
public class Auto_prolly_RED extends LinearOpMode {

    @Override
    public void runOpMode(){
        Robot robot = Robot.getInstance(this, Team.RED);
        //Start by initializing all the cameras, motors, and also the pinpoint

        //intake and transfer motor
        //DcMotor Intake = hardwareMap.dcMotor.get("intake");
        //DcMotor Transfer = hardwareMap.dcMotor.get("transfer");

        //HuskyLens huskyLens = Cameras.setupHuskyLens(this);

        //pinpoint stuff
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,9,9, AngleUnit.DEGREES,0));

        //camera
        Limelight3A limelight = robot.getLimelight();

        //driving motors
        List<DcMotor> motors = robot.getDrivingMotors();

        RevBlinkinLedDriver LED = robot.getLed();

        //put in all auto modes
        while(opModeInInit()){
            RobotUtil.placementScanner(limelight, LED);
        }

        //won't move on till u click start
        waitForStart();

        //run this code once
        if (opModeIsActive()){
            //Driving_Systems.headTo(pinpoint, limelight, motors, this, DistanceUnit.INCH, AngleUnit.DEGREES, 0, 0, -180);

        }

    }
}
