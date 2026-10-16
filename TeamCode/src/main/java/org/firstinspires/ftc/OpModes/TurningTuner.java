package org.firstinspires.ftc.OpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

import RobotUtil.*;

@Disabled
//@Autonomous
public class TurningTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = Robot.startAuto(null, this);
        //RobotUtil calc = robot.getCalc();
        //HuskyLens huskyLens = Cameras.setupHuskyLens(this);

        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        pinpoint.update();

        //Limelight3A limelight = robot.getLimelight();

        //driving motors
        //List<DcMotor> motors = robot.getDrivingMotors();

        //put in all auto modes
        //while(opModeInInit()){}

        waitForStart();

        //if (opModeIsActive()) Driving_Systems.headTo(pinpoint, limelight, motors, this, DistanceUnit.INCH, AngleUnit.DEGREES, 0, 0 , 180);
    }
}