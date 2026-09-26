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
package org.firstinspires.ftc.OpModes;

import androidx.core.math.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.List;

import RobotUtil.*;

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

    private volatile boolean locatorRunning;
    private volatile Pose2D locatorPos;
    private DcMotor Intake;
    private DcMotor Transfer;
    private HuskyLens huskyLens;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    List<Integer> brokenId;
    long lastConfirmation;
    double FRCHeading;
    double lastHeading;
    List<DcMotor> motors;
    private DcMotor Shoot;
    volatile LLResult result;
    private Robot robot;
    private RobotUtil calc;

    @Override
    public void init() {
        robot=Robot.startTele(this);

        calc = robot.getCalc();

        motors = robot.getDrivingMotors();

        brokenId = new ArrayList<>();
        //runs once as soon as "init" is pressed
        Intake = robot.getUtilMotors().get(0);
        Transfer = robot.getUtilMotors().get(1);
        Shoot = robot.getUtilMotors().get(2);

        //imu = hardwareMap.get(IMU.class, "imu");
        pinpoint = robot.getPinpoint();
        FRCHeading=0;

        limelight = robot.getLimelight();
        huskyLens = robot.getHuskyLens();
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
        locatorRunning=true;
        Thread locatorThread = new Thread(this::runCamera);
        locatorThread.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(locatorPos!=null){
            pinpoint.setPosition(locatorPos);
            locatorPos=null;
        }

        Intake.setPower(gamepad1.a ? 1 : 0);
        Transfer.setPower(gamepad1.y ? 1 : 0);
        Shoot.setPower(gamepad1.b ? 1 : 0);

        result = robot.updateLimelight();

        //Calculates how far the minimum is from the middle of the 2
        // (to know how much each should affect)
        double Difference=(CONSTANTS.MAXIMUM-CONSTANTS.MINIMUM)/2;
        //Readability of code
        double TotalTrigger=gamepad1.right_trigger+gamepad1.left_trigger;

        double speedMultiplier = (CONSTANTS.MAXIMUM-(Difference*TotalTrigger));

        if(gamepad1.start)FRCHeading=0;

        //drive variables
        double roboYaw = Math.toRadians(FRCHeading);
        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x; //controls turning

        double x = lx * Math.cos(roboYaw) + ly * Math.sin(roboYaw);
        double y = ly * Math.cos(roboYaw) - lx * Math.sin(roboYaw);

        double stickTotal = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);



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

        calc.setPowers(FLMotorPower, BLMotorPower, FRMotorPower, BRMotorPower, motors);
        lastHeading=pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
        pinpoint.update();
        FRCHeading+=(pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES)-lastHeading);
        calc.addTelemetry();

        long secs=(System.currentTimeMillis()/1000)-lastConfirmation;
        long mins=secs/60;
        if(result.isValid()){
            telemetry.addLine("Conforming Odometry :D");
        }else if (lastConfirmation>0)telemetry.addLine("Odometry Last Confirmed "+((mins>0)?(mins+"Mins and "):"")+(secs%60)+" Secs Ago");
            else telemetry.addLine("Not yet confirmed");
        telemetry.addData("stickLeftX", x);
        telemetry.addData("turn speed", rx);
        telemetry.addData("SpeedMult", speedMultiplier);
        telemetry.addData("Robot Yaw", roboYaw);
        telemetry.update();
    }

    @Override
    public void stop(){
        locatorRunning=false;
    }

    private void runCamera(){
        while (locatorRunning) {
            result = robot.updateLimelight();
            if(!result.isValid()){
                try{Thread.sleep(50);} catch (InterruptedException ignored) {}
                continue;
            }
            try {
                LLResultTypes.FiducialResult tags = calc.getBiggest(result.getFiducialResults());
                if (result.isValid() && tags.getTargetPoseCameraSpace().getOrientation().getPitch() > -70) {
                    limelight.pipelineSwitch(TagData.tagData.get(tags.getFiducialId() - 30).value);
                    while (limelight.getStatus().getPipelineIndex() == 0) Thread.sleep(20);
                    if (limelight.getStatus().getPipelineIndex() != 0) {
                        Pose3D position = limelight.getLatestResult().getBotpose_MT2();
                        locatorPos = (new Pose2D(DistanceUnit.METER, -position.getPosition().x,
                                -position.getPosition().y, AngleUnit.DEGREES,
                                calc.wrapAngle(180, -position.getOrientation().getYaw())));
                        lastConfirmation = System.currentTimeMillis();
                        limelight.pipelineSwitch(0);
                    }
                }
            } catch (Exception e) {
                locatorPos = null;
                if (limelight.getStatus().getPipelineIndex() != 0) limelight.pipelineSwitch(0);
            }
        }
    }
}
