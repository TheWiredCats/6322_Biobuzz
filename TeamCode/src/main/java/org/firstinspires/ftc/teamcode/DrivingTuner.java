package org.firstinspires.ftc.teamcode;

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
public class DrivingTuner extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    CameraConstants cc = new CameraConstants();
    PID_Systems pid = new PID_Systems();
    DcMotor FLMotor;
    DcMotor BLMotor;
    DcMotor FRMotor;
    DcMotor BRMotor;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        //driving motors
        FLMotor = hardwareMap.dcMotor.get("FL");
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BLMotor = hardwareMap.dcMotor.get("BL");
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FRMotor = hardwareMap.dcMotor.get("FR");
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BRMotor = hardwareMap.dcMotor.get("BR");
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //pinpoint, aka the odometry computer, stuff
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setPosition(new Pose2D((DistanceUnit)cc.MEASUREMENTS.get(0), -63, 63, (AngleUnit)cc.MEASUREMENTS.get(1), 0));

        pinpoint.update();

        telemetry.addData("X", pinpoint.getPosX((DistanceUnit)cc.MEASUREMENTS.get(0)));
        telemetry.addData("Y", pinpoint.getPosY((DistanceUnit)cc.MEASUREMENTS.get(0)));
        telemetry.addData("Heading", pinpoint.getHeading((AngleUnit)cc.MEASUREMENTS.get(1)));
        telemetry.update();
        waitForStart();

        if(opModeIsActive()) pid.headTo(pinpoint, limelight, List.of(FLMotor, BLMotor, FRMotor, BRMotor), this, new CameraConstants(), DistanceUnit.INCH, AngleUnit.DEGREES, 0, 0, 180);
    }
}
