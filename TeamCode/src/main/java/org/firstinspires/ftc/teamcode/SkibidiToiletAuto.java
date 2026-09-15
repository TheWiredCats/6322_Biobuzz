package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@Autonomous(name="Greedy Auto Test")
public class SkibidiToiletAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        List<DcMotor> motors = Motors.setupMotors(this);
        List<DcMotor> motorTemp = Motors.setupMotors(this);
        DcMotor Intake = motorTemp.get(0);
        DcMotor Transfer = motorTemp.get(1);
        DcMotor Shoot = motorTemp.get(2);

        GoBildaPinpointDriver pinpoint = Pinpoint.setUpPinpoint(this);
        pinpoint.setPosition(new Pose2D(CONSTANTS.unit.DU, -63, 15, CONSTANTS.unit.AU, -90));
        pinpoint.update();

        Limelight3A limelight = Cameras.setupLimeLight(this, pinpoint);
        HuskyLens huskyLens = Cameras.setupHuskyLens(this);

        waitForStart();
        if(opModeIsActive()){
            Driving_Systems.headTo(pinpoint, limelight, motors, this, CONSTANTS.unit.DU, CONSTANTS.unit.AU, -63, -15, -90);
            Intake.setPower(1);
            Transfer.setPower(1);
            sleep(2000);
            Intake.setPower(0);
            Transfer.setPower(0);
            Driving_Systems.headTo(pinpoint, limelight, motors, this, CONSTANTS.unit.DU, CONSTANTS.unit.AU, -63+(9*Math.sqrt(2)), 15, 0);
            Driving_Systems.lockIn(CONSTANTS.unit.DU, this, limelight, pinpoint, motors, 12);
        }
    }
}
