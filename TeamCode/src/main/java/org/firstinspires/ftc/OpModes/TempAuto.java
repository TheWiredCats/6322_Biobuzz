package org.firstinspires.ftc.OpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import RobotUtil.*;

@Autonomous(name = "Red auto v2.0")
public class TempAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("INIT Started");
        telemetry.update();
        sleep(1000);
        Robot robot = Robot.startAuto(Team.RED,this);
        telemetry.addLine("Robot Built");
        telemetry.update();
        sleep(1000);
        Driving_Systems ds = robot.getDSInstance();
        Limelight3A limelight = robot.getLimelight();
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        telemetry.addLine("Sub Clases Built");
        telemetry.update();
        sleep(1000);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, -63, AngleUnit.DEGREES,90));
        telemetry.addLine("Pinpoint's Position Set");
        waitForStart();
        //ds.headTo(pinpoint, limelight, this, DistanceUnit.INCH, AngleUnit.DEGREES, 0, -50, 90, PIDModes.DRIVING);
        //ds.headTo(pinpoint, limelight, this, DistanceUnit.INCH, AngleUnit.DEGREES, -55, -50, 180, PIDModes.TURNING);
        //ds.headTo(pinpoint, limelight, this, DistanceUnit.INCH, AngleUnit.DEGREES, -55, -63, 90, PIDModes.TURNING);
    }
}
