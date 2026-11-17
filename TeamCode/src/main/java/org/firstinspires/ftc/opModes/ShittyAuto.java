package org.firstinspires.ftc.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import robotUtil.*;
@Autonomous(name = "Bad_Auto")
public class ShittyAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = Robot.startAuto(Team.RED, this);
        RobotUtil calc = robot.getCalc();
        waitForStart();
         calc.setPowers(0.25, -0.25, -0.25, 0.25);
         sleep(500);
         calc.brakeMotors();
         sleep(2000);
        while(opModeIsActive()){
            calc.setPowers(0.25,0.25,0.25,0.25);
        }
        calc.brakeMotors();
    }
}
