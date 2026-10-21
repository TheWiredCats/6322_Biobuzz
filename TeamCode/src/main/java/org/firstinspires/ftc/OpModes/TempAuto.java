package org.firstinspires.ftc.OpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import RobotUtil.*;

@Autonomous(name = "Red auto v2.0")
public class TempAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = Robot.startAuto(Team.RED,this);
        //Driving_Systems ds = robot.getDSInstance();
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, -63, AngleUnit.DEGREES,90));
        waitForStart();
        //ds.headTo(pinpoint, this, DistanceUnit.INCH, AngleUnit.DEGREES, 0, -50, 90, PIDModes.DRIVING);
        //ds.headTo(pinpoint, this, DistanceUnit.INCH, AngleUnit.DEGREES, -55, -50, 180, PIDModes.TURNING);
        //ds.headTo(pinpoint, this, DistanceUnit.INCH, AngleUnit.DEGREES, -55, -63, 90, PIDModes.TURNING);
    }
}
