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

public class MTYCTWD9110 extends OpMode {
    boolean buttonDown=false;

    double dBearing = 0;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor cannonOne = null;
    private DcMotor cannonTwo = null;
    private DcMotor intakeMotor = null;


    //Declare the Servos
    //Make sure the ID's also math your configuration on the Drive Hub
    private CRServo launchOne = null;
    private CRServo launchTwo = null;

    private IMU imu = null;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        cannonOne = hardwareMap.dcMotor.get("cannonOne");
        cannonTwo = hardwareMap.dcMotor.get("cannonTwo");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        cannonOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cannonTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        //Setting variables for the servos
        launchOne = hardwareMap.get(CRServo.class, "launchOne");
        launchTwo = hardwareMap.get(CRServo.class, "launchTwo");


        //Reverses direction of left motors for easier coding/math
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
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
        double speedmult = 1;

        double power=0.8;
        double powerReverse=-0.8;
        double powerOff=0;


        //Button to turn on cannon
        if(gamepad1.right_bumper){
            cannonOne.setPower(power);
            cannonTwo.setPower(powerReverse);
        }else if(!gamepad1.dpad_down){
            cannonOne.setPower(powerOff);
            cannonTwo.setPower(powerOff);
        }


        //To intake from ground
        if(gamepad1.a){
            intakeMotor.setPower(powerReverse);
        }else if (!gamepad1.x){
            intakeMotor.setPower(powerOff);
        }

        //Turns on the servos to move it up
        //After a quarter of a second the arm turns on
        if(gamepad1.b){
            launchOne.setPower(power);
            launchTwo.setPower(powerReverse);
        }else if(!gamepad1.dpad_down){
            launchOne.setPower(powerOff);
            launchTwo.setPower(powerOff);
        }

        //To take an artifact out of the cannons/drop it
        //Used for patterns
        if(gamepad1.dpad_down){
            launchOne.setPower(powerReverse);
            launchTwo.setPower(power);
            cannonOne.setPower(powerReverse);
            cannonTwo.setPower(power);
        }else if(!gamepad1.right_bumper) {
            cannonOne.setPower(powerOff);
            cannonTwo.setPower(powerOff);
        }else if (!gamepad1.b){
            launchOne.setPower(powerOff);
            launchTwo.setPower(powerOff);
        }

        //Starts inputting into the robot
        if(gamepad1.x){
            intakeMotor.setPower(power);
        }else if (!gamepad1.a){
            intakeMotor.setPower(powerOff);
        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if(gamepad1.right_stick_button)buttonDown=false;
        if(!buttonDown&&!gamepad1.right_stick_button) {
            buttonDown = true;
            slowspeed = !slowspeed;
        }

        speedmult=(slowspeed)?0.5:((1-gamepad1.left_trigger<=0.5)?0.5:(1-gamepad1.left_trigger));


        double roboYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        if (currentDetections.isEmpty()) {
            telemetry.addLine("No AprilTags visible!");
        } else {

            for (AprilTagDetection detection : currentDetections) {
                if (detection.ftcPose != null) {
                    telemetry.addData("TagID", detection.id);
                    telemetry.addData("Bearing: ", detection.ftcPose.bearing);
                    telemetry.addData("Range: ", detection.ftcPose.range);
                    if (gamepad1.left_bumper) {
                        rx = dBearing / 35;
                    }
                } else {
                    telemetry.addLine("Tag detected, but pose estimate is null.");
                }
            }
        }

        double x = lx * Math.cos(roboYaw) + ly * Math.sin(roboYaw);
        double y = ly * Math.cos(roboYaw) - lx * Math.sin(roboYaw);

        double stickTotal = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        double FLMotorPower = ((y + x + rx) / stickTotal) * speedmult;
        double FRMotorPower = ((y - x - rx) / stickTotal) * speedmult;
        double BLMotorPower = ((y - x + rx) / stickTotal) * speedmult;
        double BRMotorPower = ((y + x - rx) / stickTotal) * speedmult;

        frontLeftMotor.setPower(FLMotorPower);
        frontRightMotor.setPower(FRMotorPower);
        backLeftMotor.setPower(BLMotorPower);
        backRightMotor.setPower(BRMotorPower);

        if (gamepad1.right_stick_button)
            imu.resetYaw();

        telemetry.update();
    }

    //private void setLauncherRPM(double topSpeedTarget, double bottomSpeedTarget){
    //    Trpm = (topLauncher.getVelocity() / 537.7) * 60;
    //    DTrpm = topSpeedTarget - Trpm;

    //    Brpm = (topLauncher.getVelocity() / 537.7) * 60;
    //    DBrpm = bottomSpeedTarget - Brpm;

    //    ITrpm += DTrpm;
    //    IBrpm += DBrpm;

    //    topLauncher.setPower(((DTrpm * 1) + (ITrpm * 0.1) / 150) * -1);
    //    bottomLauncher.setPower(((DBrpm * 1) + (IBrpm * 0.1) / 215) * -1);

    //}

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
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Cam"));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal.
        visionPortal = builder.build();

    }
}
