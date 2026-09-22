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
public class TurningTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance(this , null);
        //HuskyLens huskyLens = Cameras.setupHuskyLens(this);

        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        pinpoint.update();

        Limelight3A limelight = robot.getLimelight();

        //driving motors
        List<DcMotor> motors = robot.getDrivingMotors();

        RevBlinkinLedDriver LED = robot.getLed();

        //put in all auto modes
        while(opModeInInit()){
            RobotUtil.placementScanner(limelight, LED);
        }

        waitForStart();

        //if (opModeIsActive()) Driving_Systems.headTo(pinpoint, limelight, motors, this, DistanceUnit.INCH, AngleUnit.DEGREES, 0, 0 , 180);
    }
}