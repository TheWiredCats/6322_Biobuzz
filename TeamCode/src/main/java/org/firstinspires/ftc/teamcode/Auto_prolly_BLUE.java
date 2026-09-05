package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@Autonomous
public class Auto_prolly_BLUE extends LinearOpMode {
    PID_Systems pid = new PID_Systems();
    CameraConstants cc = new CameraConstants();
    @Override
    public void runOpMode(){
        //Start by initializing all the cameras, motors, and also the pinpoint

        //intake and transfer motor
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor transfer = hardwareMap.dcMotor.get("transfer");

        //camera 1 and 2
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        //pinpoint, aka the odometry computer, stuff
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,63, AngleUnit.DEGREES,0));
        pinpoint.update();

        //driving motors
        DcMotor FLMotor = hardwareMap.dcMotor.get("FL");
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor BLMotor = hardwareMap.dcMotor.get("BL");
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor FRMotor = hardwareMap.dcMotor.get("FR");
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor BRMotor = hardwareMap.dcMotor.get("BR");
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        List<DcMotor> motors = List.of(FLMotor, BLMotor, FRMotor, BRMotor);

        //won't move on till u click start
        waitForStart();

        //run this code once
        if (opModeIsActive()){
            //drive to top left corner
            pid.driveTo(DistanceUnit.INCH, pinpoint, limelight, motors, this, cc, 60, 60);
            sleep(1000);

            //drive to top right corner
            pid.driveTo(DistanceUnit.INCH, pinpoint, limelight, motors, this, cc, 60, -60);
            sleep(1000);

            //drive to bottom right corner
            pid.driveTo(DistanceUnit.INCH, pinpoint, limelight, motors, this, cc, -60, -60);
            sleep(1000);

            //drive back to start
            pid.driveTo(DistanceUnit.INCH, pinpoint, limelight, motors, this, cc, -60,60);
            sleep(1000);

            //turn to the right
            pid.turnTo(AngleUnit.DEGREES,this, pinpoint,motors,-45);

            //look for an April tag and lock move until ur exactly 3ft away from it
            pid.lockIn(DistanceUnit.INCH,this, limelight, pinpoint,new CameraConstants(),motors,36);
        }

    }
}
