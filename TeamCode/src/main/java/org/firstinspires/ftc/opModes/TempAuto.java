package org.firstinspires.ftc.opModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import robotUtil.*;

@Autonomous(name = "Red auto v2.0")
public class TempAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = Robot.startAuto(Team.RED,this);
        Driving_Systems ds = robot.getDSInstance();
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        waitForStart();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0));
        ds.headTo(pinpoint, this, DistanceUnit.INCH, AngleUnit.DEGREES, 48, 0, 180, PIDModes.TURNING);
    }
}
