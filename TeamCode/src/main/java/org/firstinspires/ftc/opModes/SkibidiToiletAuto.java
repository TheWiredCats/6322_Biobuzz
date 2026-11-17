package org.firstinspires.ftc.opModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

import robotUtil.*;

@Autonomous(name="Greedy Auto BLUE")
public class SkibidiToiletAuto extends LinearOpMode {

    @Override
    public void runOpMode(){
        Robot robot = Robot.startAuto(Team.BLUE, this);
        Driving_Systems ds = robot.getDSInstance();
        List<DcMotor> motorTemp = robot.getUtilMotors();
        DcMotor intake = motorTemp.get(0);
        DcMotor transfer = motorTemp.get(1);
        DcMotor shoot = motorTemp.get(2);

        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        pinpoint.setPosition(new Pose2D(Constants.unit.DU, -63, 24, Constants.unit.AU, 180));
        pinpoint.update();

        Limelight3A limelight = robot.getLimelight();

        waitForStart();

        ds.headTo(pinpoint, this, Constants.unit.DU, Constants.unit.AU, -48, 12, 180, PIDModes.DRIVING);
        shoot.setPower(1);
        sleep(100);
        transfer.setPower(1);
        sleep(2000);
        shoot.setPower(0);
        transfer.setPower(0);
        ds.headTo(pinpoint, this, Constants.unit.DU, Constants.unit.AU, -48,48, 180, PIDModes.DRIVING);
        ds.headTo(pinpoint, this, Constants.unit.DU, Constants.unit.AU, 48, 48, 180, PIDModes.DRIVING);
        ds.headTo(pinpoint, this, Constants.unit.DU, Constants.unit.AU, 48, 63, 180, PIDModes.DRIVING);
    }
}
