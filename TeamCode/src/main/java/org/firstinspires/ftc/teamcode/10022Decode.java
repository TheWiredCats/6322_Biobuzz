/*
Copyright 2026 FIRST Tech Challenge Team 10022

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When an selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class Decode2 extends OpMode {
    boolean buttonDown=false;

    double dBearing = 0;

    double Trpm = 0;
    double Brpm = 0;
    double DTrpm = 0;
    double DBrpm = 0;

    double ITrpm = 0;
    double IBrpm = 0;

    /* Declare OpMode members. */

    private DcMotorEx bottomLauncher = null;
    private DcMotorEx topLauncher = null;
    private DcMotor Intake = null;

    private DcMotor FLMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BRMotor = null;

    private IMU imu = null;

    private CRServo s1 = null;
    private CRServo s2 = null;
    private CRServo s3 = null;
    private CRServo s4 = null;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        bottomLauncher = hardwareMap.get(DcMotorEx.class,"bottomLaunch");
        topLauncher = hardwareMap.get(DcMotorEx.class,"topLaunch");
        Intake = hardwareMap.dcMotor.get("intake");

        imu = hardwareMap.get(IMU.class, "imu");

        FLMotor = hardwareMap.dcMotor.get("FLMotor");
        BLMotor = hardwareMap.dcMotor.get("BLMotor");
        FRMotor = hardwareMap.dcMotor.get("FRMotor");
        BRMotor = hardwareMap.dcMotor.get("BRMotor");

       // BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        s1 = hardwareMap.get(CRServo.class, "S1");
        s2 = hardwareMap.get(CRServo.class, "S2");
        s3 = hardwareMap.get(CRServo.class, "S3");
        s4 = hardwareMap.get(CRServo.class, "S4");


        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
            )
        );

        initAprilTag();
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        boolean slowspeed = false;
        boolean reversemode = false;
        double speedmult = 1;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        Intake.setPower(gamepad1.a?(reversemode?1:-1):0);

        reversemode=gamepad1.b;

        Stark(gamepad1.x,reversemode);

        topLauncher.setPower(gamepad1.right_trigger>=0.5?(reversemode?1:-1)*gamepad1.right_trigger:0);
        bottomLauncher.setPower(gamepad1.right_trigger>=0.5?(reversemode?-1:1)*gamepad1.right_trigger:0);

        if(gamepad1.right_stick_button)buttonDown=false;
        if(!buttonDown&&!gamepad1.right_stick_button) {
            buttonDown = true;
            slowspeed = !slowspeed;
        }

        speedmult=(slowspeed)?0.5:((1-gamepad1.left_trigger<=0.5)?0.5:(1-gamepad1.left_trigger));


        double roboYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            for (AprilTagDetection detection : currentDetections) if (detection.metadata != null)dBearing = detection.ftcPose.bearing;
            rx = dBearing / 35;
        }

        if (gamepad1.right_bumper) {
            setLauncherRPM(130, 130);
        } else {
            ITrpm = 0;
            IBrpm = 0;
        }

        double x = lx * Math.cos(roboYaw) + ly * Math.sin(roboYaw);
        double y = lx * Math.sin(-roboYaw) + ly * Math.cos(roboYaw);

        double stickTotal = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        double FLMotorPower = ((y + x + rx) / stickTotal) * speedmult;
        double FRMotorPower = ((y + x - rx) / stickTotal) * speedmult;
        double BLMotorPower = ((y - x + rx) / stickTotal) * speedmult;
        double BRMotorPower = ((y - x - rx) / stickTotal) * speedmult;

        FLMotor.setPower(FLMotorPower);
        FRMotor.setPower(FRMotorPower);
        BLMotor.setPower(BLMotorPower);
        BRMotor.setPower(BRMotorPower);

        if (gamepad1.right_stick_button)
            imu.resetYaw();

        telemetry.addData("Slowmode", slowspeed);
        telemetry.addData("SpeedMult", speedmult);
        telemetry.addData("Yaw", roboYaw);
        telemetry.addData("Reversemode", reversemode);
        telemetry.addData("Shootingpower", gamepad1.right_trigger);
        telemetry.addData("bottomPower", (((DBrpm * 1) + (IBrpm * 0.1)) / 215) * -1);

        // max bottom 215 rpm
        telemetry.addData("Bottom Rpm:", (bottomLauncher.getVelocity() / 537.7) * 60);
        // max top 150 rpm
        telemetry.addData("Top Rpm:", (topLauncher.getVelocity() / 537.7) * 60);

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addData("Bearing: ", detection.ftcPose.bearing);
                telemetry.addData("Range: ", detection.ftcPose.range);
            }
        }
    }

    private void Stark(boolean on, boolean power){
        s1.setPower(on?(power?1:-1):0);
        s2.setPower(on?(power?1:-1):0);
        s3.setPower(on?(power?-1:1):0);
        s4.setPower(on?(power?-1:1):0);
    }
    private void setLauncherRPM(double topSpeedTarget, double bottomSpeedTarget){
            Trpm = (topLauncher.getVelocity() / 537.7) * 60;
            DTrpm = topSpeedTarget - Trpm;

            Brpm = (topLauncher.getVelocity() / 537.7) * 60;
            DBrpm = bottomSpeedTarget - Brpm;

            ITrpm += DTrpm;
            IBrpm += DBrpm;

            topLauncher.setPower(((DTrpm * 1) + (ITrpm * 0.1) / 150) * -1);
            bottomLauncher.setPower(((DBrpm * 1) + (IBrpm * 0.1) / 215) * -1);

    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

            .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Cam"));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal.
        visionPortal = builder.build();

    }   // end method initAprilTag()
}
