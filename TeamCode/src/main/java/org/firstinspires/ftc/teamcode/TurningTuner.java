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

@Autonomous
public class TurningTuner extends LinearOpMode {

    @Override
    public void runOpMode() {

        //HuskyLens huskyLens = Cameras.setupHuskyLens(this);

        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = Pinpoint.setUpPinpoint(this);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        pinpoint.update();

        Limelight3A limelight = Cameras.setupLimeLight(this, pinpoint);

        //driving motors
        List<DcMotor> motors = Motors.setupDrivingMotors(this);

        RevBlinkinLedDriver LED = Led.LEDSetUP(this);

        //put in all auto modes
        while(opModeInInit()){
            Led.placementScanner(limelight, LED);
        }

        waitForStart();

        if (opModeIsActive()) Driving_Systems.headTo(pinpoint, limelight, motors, this, DistanceUnit.INCH, AngleUnit.DEGREES, 0, 0 , 180);
    }
}