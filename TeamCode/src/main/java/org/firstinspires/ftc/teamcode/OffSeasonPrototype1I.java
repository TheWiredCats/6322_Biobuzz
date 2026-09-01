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

import androidx.core.math.MathUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.text.Normalizer;
import java.util.concurrent.TimeUnit;

import java.util.List;

/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When a selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp(name = "OffSeason Prototype 1I")

public class OffSeasonPrototype1I extends OpMode {
    /* Declare OpMode members. */
    //as soon as teleop selected

    private DcMotor Intake = null;
    private DcMotor Transfer = null;
    private DcMotor FLMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BRMotor = null;
    private HuskyLens huskyLens = null;

    private Limelight3A limelight = null;

//    private IMU imu = null;

    private GoBildaPinpointDriver pinpoint;
    //private Deadline rateLimit = null;

    //the public ones are so we can accsess them in the autos
    //really all they are is to only have to do this stuff once
    double currentY=0;
    double currentX=0;
    boolean codeMissing;
    int brokenId;
    int lastConfirmation;
    double FRCHeading=0;
    int id;
    double ZDifference;
    double LRDifference;
    CameraConstants cc = new CameraConstants();
    //get freshies to fill with position and direction of apriltags on field after kickoff
    //Should be in the format {x cord, y cord, facing direction} (0 for +x,1 for -x,2 for +y, 3 for-y)
    // the cords should also be in inches
    //first array should be empty so u could use fiducial id directly

    double lastHeading;

    private void addPinpointTelemetry(){
        lastHeading=pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
        pinpoint.update();
        FRCHeading+= pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES)-lastHeading;
        double xVel=pinpoint.getVelX(DistanceUnit.INCH);
        double yVel=pinpoint.getVelY(DistanceUnit.INCH);
        telemetry.addData("Direction",
                ((Math.abs(xVel)<2)?(Math.abs(yVel)<2?"Not moving":""):xVel>0?"Forward":"Backwards")
                        +((xVel==0&&yVel==0)?"":" and ")
                        +(Math.abs(yVel)<2?"":yVel<0?"Right":"Left"));
        telemetry.addData("Heading", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("X position", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y position", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("2D Position", pinpoint.getPosition());
    }

    @Override
    public void init() {
        //runs once as soon as "init" is pressed
        Intake = hardwareMap.dcMotor.get("intake");
        Transfer = hardwareMap.dcMotor.get("transfer");

//        imu = hardwareMap.get(IMU.class, "imu");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //swapped reversed and forward directions -Noah Randall 8/24 2:15pm
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        FLMotor = hardwareMap.dcMotor.get("FL");
        BLMotor = hardwareMap.dcMotor.get("BL");
        FRMotor = hardwareMap.dcMotor.get("FR");
        BRMotor = hardwareMap.dcMotor.get("BR");

        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FLMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FRMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        limelight.pipelineSwitch(0);
        //Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        //rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData("HL:", "Problem communicating with " + huskyLens.getDeviceName());
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        pinpoint.recalibrateIMU();
        /*
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
            )
        );
        */
    }
    /*
    *Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        codeMissing=false;
        //move to start of auto in start() when have it
        pinpoint.resetPosAndIMU();
        //imu.resetYaw();
        limelight.start();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //rateLimit.reset();

        Intake.setPower(gamepad1.a?-1:0);
        Transfer.setPower(gamepad1.y?1:0);

        //how low/high the Speed can go with both triggers down/up respectully
        final double MINIMUM = 0.25;
        //maximum must be less than or equal to 1
        final double MAXIMUM = 1;

        //Calculates how far the minimum is from the middle of the 2
        // (to know how much each should affect)
        double Difference=(MAXIMUM-MINIMUM)/2;

        //Readability of code
        double TotalTrigger=gamepad1.right_trigger+gamepad1.left_trigger;

        double speedMultiplier = (MAXIMUM-(Difference*TotalTrigger));

        lastHeading=pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        FRCHeading+=(pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES)-lastHeading);

        if(gamepad1.start)FRCHeading=0;

        //drive variables
        double roboYaw = Math.toRadians(FRCHeading);
        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x; //controls turning

        double x = lx * Math.cos(roboYaw) + ly * Math.sin(roboYaw);
        double y = ly * Math.cos(roboYaw) - lx * Math.sin(roboYaw);

        double stickTotal = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);


        LLResult result = limelight.getLatestResult(); //april tag code
        if (result.isValid()) {
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            //double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("LL: April tag", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                /*
                // rx = fr.getTargetPoseRobotSpace().getOrientation().getYaw() / 35; it wasn't one line of code

                // If Limelight is mounted forward, tx IS your error.
                // You might need to flip the sign depending on your motor configuration.


                //Makes sure that the heading error is between -180 and 180 so the robot doesn't spin violently
                double headingError;
                if(result.getTx()%360==180){
                    double sign = Math.signum(result.getTx());
                    headingError = sign * 180;
                }else{
                    double adjustedError=result.getTx()+180;
                    double fixedAdjustedError=adjustedError%360;
                    headingError=fixedAdjustedError-180;
                }

                 Simple Proportional control (P-loop). Adjust Kp until it snaps to target smoothly.
                double Kp = 0.04;

                 Optional: Cap rx so it doesn't spin violently
                rx = headingError * Kp; rx = MathUtils.clamp(headingError*Kp,-0.5,0.5);
                */

                // get rid of -20 after kickoff
                id = fr.getFiducialId() - 20;
                if (id>=0&&id<cc.APRIL_TAG_POSITIONS.length) {
                    double tx = -fr.getTargetXDegrees();
                    double ty = -fr.getTargetYDegrees();
                    if (cc.APRIL_TAG_POSITIONS[id][2] >= 0 && cc.APRIL_TAG_POSITIONS[id][2] <= 3) {
                        lastConfirmation = (int) (System.currentTimeMillis() / 1000);
                    }
                    if (Math.abs(tx) < 60 && Math.abs(ty) < 60) {
                        double apriltagX = cc.APRIL_TAG_POSITIONS[id][0];
                        double apriltagY = cc.APRIL_TAG_POSITIONS[id][1];
                        //how far away the april tag is
                        ZDifference = cc.APRIL_TAG_HEIGHT / Math.tan(Math.toRadians(ty));
                        //how far left or right it is, negative is left and right is positive
                        LRDifference = ZDifference * Math.tan(Math.toRadians(tx));
                        switch ((int) cc.APRIL_TAG_POSITIONS[id][2]) {
                            case (0):
                                currentX = apriltagX - ZDifference;
                                currentY = apriltagY - LRDifference;
                                break;
                            case (1):
                                currentX = apriltagX + ZDifference;
                                currentY = apriltagY + LRDifference;
                                break;
                            case (2):
                                currentX = apriltagX + LRDifference;
                                currentY = apriltagY - ZDifference;
                                break;
                            case (3):
                                currentX = apriltagX - LRDifference;
                                currentY = apriltagY + ZDifference;
                                break;
                            default:
                                codeMissing = true;
                                brokenId = id;
                                currentX = pinpoint.getPosX(DistanceUnit.INCH) - cc.CAMERA_X_OFFSET;
                                currentY = pinpoint.getPosY(DistanceUnit.INCH) - cc.CAMERA_Y_OFFSET;
                                break;
                        }
                        if (!codeMissing) telemetry.addLine("Code Working!");
                        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, currentX + cc.CAMERA_X_OFFSET, currentY + cc.CAMERA_Y_OFFSET, AngleUnit.DEGREES, pinpoint.getHeading(AngleUnit.DEGREES)));
                    }

                }
            }
        }

        HuskyLens.Block[] blocks = huskyLens.blocks(); //huskylens code
        telemetry.addData("HL Block Count", blocks.length);

        if (gamepad1.x) for (HuskyLens.Block block : blocks) {
            telemetry.addData("HL:", block.toString());

            // HuskyLens Constants
            final double SCREEN_CENTER_X = 160.0;
            final double HALF_HORIZONTAL_FOV_RAD = Math.toRadians(27.5); // 55 degrees / 2

            // 1. Get your 2D data from HuskyLens
            int blockX = block.x;

            // 2. Calculate your 3D Z-distance first (from previous step)
            double distanceZ = (2.9 * 4.6) / block.width;

            // 3. Calculate pixel offset from screen center
            double pixelOffset = blockX - SCREEN_CENTER_X;

            // 4. Normalize the offset (-1.0 to 1.0) and convert to radians
            double angleX = (pixelOffset / SCREEN_CENTER_X) * HALF_HORIZONTAL_FOV_RAD;

            // 5. Calculate final physical X position (in inches or cm depending on your Z unit)
            double positionX = distanceZ * Math.tan(angleX);

            // Simple Proportional control (P-loop). Adjust Kp until it snaps to target smoothly.
            final double Kp = 1.5;
            //rx = headingError * Kp;
            rx = MathUtils.clamp(positionX * Kp, -0.5, 0.5);


            telemetry.addData("HL 3D Z (Distance)", distanceZ);
            telemetry.addData("HL 3D X (Lateral)", positionX);

            /*
             * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
             * - blocks[i].width and blocks[i].height   (size of box, in pixels)
             * - blocks[i].left and blocks[i].top       (edges of box)
             * - blocks[i].x and blocks[i].y            (center location)
             * - blocks[i].id                           (Color ID)
             *
             * These values have Java type int (integer).
             */
        }

        double FLMotorPower = ((y + x + rx) / stickTotal) * speedMultiplier;
        double BLMotorPower = ((y - x + rx) / stickTotal) * speedMultiplier;
        double FRMotorPower = ((y - x - rx) / stickTotal) * speedMultiplier;
        double BRMotorPower = ((y + x - rx) / stickTotal) * speedMultiplier;

        FLMotor.setPower(FLMotorPower);
        FRMotor.setPower(FRMotorPower);
        BLMotor.setPower(BLMotorPower);
        BRMotor.setPower(BRMotorPower);
        int secs=((int)(System.currentTimeMillis()/1000))-lastConfirmation;
        int mins=secs/60;
        if(codeMissing)telemetry.addLine("CODE MISSING for "+ brokenId + "!!!!!!!");
        if(result.isValid()){
            double Facing = cc.APRIL_TAG_POSITIONS[id][2];
            telemetry.addLine("Conforming Odometry :D");
            telemetry.addData("Tag Data","Looking at Tag: %d, X Position: %.2f, Y Position: %.2f, Facing: %s",id,cc.APRIL_TAG_POSITIONS[id][0],cc.APRIL_TAG_POSITIONS[id][1],
                    (Facing>=2?(Facing==2?"+Y":"-Y"):(Facing==0?"+X":"-X")));
        }else if (lastConfirmation>0)telemetry.addLine("Odometry Last Confirmed "+((mins>0)?(mins+"Mins and "):"")+(secs%60)+" Secs Ago");
            else telemetry.addLine("Not yet confirmed");
        addPinpointTelemetry();
        telemetry.addData("stickleftX", x);
        telemetry.addData("turn speed", rx);
        //after the camera code rx might have been altered
        telemetry.addData("SpeedMult", speedMultiplier);
        telemetry.addData("Robot Yaw", roboYaw);
        LLStatus status = limelight.getStatus();
        telemetry.addData("LL STATS", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("LL Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());
        telemetry.update();
    }
}
