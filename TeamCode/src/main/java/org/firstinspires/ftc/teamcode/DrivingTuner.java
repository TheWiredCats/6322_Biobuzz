package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

import RobotUtil.CONSTANTS;
import RobotUtil.Robot;
import RobotUtil.RobotUtil;

@Autonomous
public class DrivingTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance(this, null);

        //HuskyLens huskyLens = Cameras.setupHuskyLens(this);

        //driving motors
        List<DcMotor> motors = robot.getDrivingMotors();



        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        pinpoint.setPosition(new Pose2D(CONSTANTS.unit.DU, -63, 63, CONSTANTS.unit.AU, 0));
        pinpoint.update();

        Limelight3A limelight = robot.getLimelight();

        RevBlinkinLedDriver LED = robot.getLed();

        //put in all auto modes
        while(opModeInInit()){
            RobotUtil.placementScanner(limelight, LED);
        }

        waitForStart();

        //if(opModeIsActive()) Driving_Systems.goTo(pinpoint, limelight, motors, this, CONSTANTS.unit.DU, 21, 12);
    }
}
