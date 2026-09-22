package org.firstinspires.ftc.OpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

import RobotUtil.CONSTANTS;
import RobotUtil.Driving_Systems;
import RobotUtil.PIDModes;
import RobotUtil.Robot;
import RobotUtil.Team;

@Autonomous(name="Greedy Auto Test")
public class SkibidiToiletAuto extends LinearOpMode {

    @Override
    public void runOpMode(){
        Robot robot = Robot.getInstance(this, Team.RED);
        Driving_Systems ds = robot.getDSInstance();
        List<DcMotor> motors = robot.getDrivingMotors();
        List<DcMotor> motorTemp = robot.getUtilMotors();
        DcMotor intake = motorTemp.get(0);
        DcMotor transfer = motorTemp.get(1);
        DcMotor shoot = motorTemp.get(2);

        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        pinpoint.setPosition(new Pose2D(CONSTANTS.unit.DU, -63, 15, CONSTANTS.unit.AU, -90));
        pinpoint.update();

        Limelight3A limelight = robot.getLimelight();

        waitForStart();
        if(opModeIsActive()){
            ds.headTo(pinpoint, limelight, motors, this, CONSTANTS.unit.DU, CONSTANTS.unit.AU, -63, -15, -90, PIDModes.DRIVING);
            intake.setPower(1);
            transfer.setPower(1);
            sleep(2000);
            intake.setPower(0);
            transfer.setPower(0);
            ds.headTo(pinpoint, limelight, motors, this, CONSTANTS.unit.DU, CONSTANTS.unit.AU, -63+(9*Math.sqrt(2)), 15, 0, PIDModes.DRIVING);
            ds.lockIn(CONSTANTS.unit.DU, this, limelight, pinpoint, motors, 18);
            shoot.setPower(1);
            sleep(1000);
            shoot.setPower(0);
        }
    }
}
