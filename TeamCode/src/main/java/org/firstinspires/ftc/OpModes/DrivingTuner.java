package org.firstinspires.ftc.OpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

import RobotUtil.*;

@Autonomous
public class DrivingTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = Robot.startAuto(null, this);
        RobotUtil calc = robot.getCalc();

        //HuskyLens huskyLens = Cameras.setupHuskyLens(this);

        //driving motors
        List<DcMotor> motors = robot.getDrivingMotors();



        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        pinpoint.setPosition(new Pose2D(CONSTANTS.unit.DU, -63, 63, CONSTANTS.unit.AU, 0));
        pinpoint.update();

        //put in all auto modes
        while(opModeInInit()){
            calc.placementScanner();
        }

        waitForStart();

        //if(opModeIsActive()) Driving_Systems.goTo(pinpoint, limelight, motors, this, CONSTANTS.unit.DU, 21, 12);
    }
}
